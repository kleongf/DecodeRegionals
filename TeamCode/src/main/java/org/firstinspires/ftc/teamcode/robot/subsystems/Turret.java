package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

public class Turret extends Subsystem {
    public DcMotorEx turretMotor;
    public PIDFController turretController;
    public double target = 0;
    private double ticksPerRevolution = 1381; // 383.6*5, (5 to 1) now 383.6 * (90/25) (90 to 25) = 1381
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    private double feedforward = 0;

    private double offset = 0;
    private double maxPower = 0.7;
    private double kS = 0.035;


    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //turretController = new PIDFController(0.005, 0, 0.00005, 0);
        // TODO: retune turret. doesn't need to be strong just needs to follow goal
        turretController = new PIDFController(0.002, 0, 0.00012, 0);
    }

    @Override
    public void update() {

        double c = turretMotor.getCurrentPosition();
                // - offset/ticksPerRadian;
        double t = weirdAngleWrap(target) * ticksPerRadian;

        double power = turretController.calculate(c, t);
        double error = t-c;
        power += kS * Math.signum(error); // kS so that it works better, lots of friction but this is
        // currently a random number that must be tuned. should work better for now though
        if (Math.abs(t-c) > 300 && Math.abs(power) > maxPower) {
            power = Math.signum(power) * maxPower;
        }

//        if (Math.abs(c-t) > 10) {
//            double error = t-c;
//            power += 0.01 * Math.signum(error);
//        }

//        if (Math.abs(power) > maxPower) {
//            power = maxPower * Math.signum(power);
//        }

        // power += feedforward;
        turretMotor.setPower(power);
    }

    public void setPDCoefficients(double p, double d) {
        turretController.setPIDF(p, 0, d, 0);
    }
    public void setKs(double x) {kS = x;}

    @Override
    public void start() {

    }

    public void setFeedforward(double x) {
        feedforward = x;
    }
    public double getTarget() {
        return weirdAngleWrap(target) * ticksPerRadian;
    }
    public double getCurrent() {
        return turretMotor.getCurrentPosition();
    }

    public double weirdAngleWrap(double radians) {
        while (radians > 0) {
            radians -= 2 * Math.PI;
        }
        while (radians < -2 * Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double x) {
        target = x;
    }
    public void setOffset(double x) {
        offset = x;
    }
    public boolean atTarget(double threshold) {
        return Math.abs(turretMotor.getCurrentPosition()-weirdAngleWrap(target) * ticksPerRadian) < threshold;
    }
}
