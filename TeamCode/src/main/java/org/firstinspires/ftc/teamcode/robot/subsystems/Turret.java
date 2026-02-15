package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

public class Turret extends Subsystem {
    public DcMotorEx turretMotor;
    public PIDFController turretController;
    public double target = 0;
    private double ticksPerRevolution = 1381; // 383.6*5, (5 to 1) now 383.6 * (90/25) (90 to 25) = 1381
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    private double feedforward = 0;

    private double offset = 0;
    private double maxPower = 0.5;
    private double kS = 0.01;
    public static double encoderOffsetDegrees = -120;
    public static double maxVoltage = 3.29;
    private double encoderGearRatio = 17/14d;
    private AnalogInput externalEncoder;


    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        externalEncoder = hardwareMap.get(AnalogInput.class, "externalEncoder");


        //turretController = new PIDFController(0.005, 0, 0.00005, 0);
        // TODO: retune turret. doesn't need to be strong just needs to follow goal
        turretController = new PIDFController(0.005, 0, 0.000, 0);
    }

    // the turret does not tend to drift too much
    // how can we tell what rotation the encoder is on?

    // if the turret says 180 deg for example, encoder would have turned (90/25)/2 times
    // we can take the floormod of this number: 90/50 = 1, know it has rotated once
    // recalculate the error by using the rest of the encoder value

    // basically, turns made = floorMod(#ticks/383.6) + externalencoder to ticks
    // problem is when turns made is between a number, say 1 and 2, and
    // external encoder is about the same because the turret encoder drifted

    // we could calculate a value and take the value between ticks. if the error is
    // very large (values are very different) then ignore it

    // problem is that during this time, sotm and normal shooting would not work

    // is this foolproof?

    //
    // ehhh this might not be as good but it should be fine

    private double calculatePositionTicks(double voltage) {
        double realOffset = Math.toRadians(encoderOffsetDegrees + 360); // added to final

        // if it's reversed

        double position = ((voltage /  maxVoltage) * 2 * Math.PI) % (2 * Math.PI) + realOffset;
        double posToTicksGeared = position * ticksPerRadian * encoderGearRatio;

        return posToTicksGeared - 2 * ticksPerRevolution; // + ticksPerRevolution * rotations;
    }

    // 13.1, 59.7, 148
    // 16, 70
    // 30, 135.5

    @Override
    public void update() {
        double c = calculatePositionTicks(externalEncoder.getVoltage());

        // double c = turretMotor.getCurrentPosition();
                // - offset/ticksPerRadian;
        double t = weirdAngleWrap(target) * ticksPerRadian;

        double power = turretController.calculate(c, t);
        double error = t-c;
        power += kS * Math.signum(error); // kS so that it works better, lots of friction but this is
        // currently a random number that must be tuned. should work better for now though
        if (Math.abs(t-c) > 300 && Math.abs(power) > maxPower) {
            power = Math.signum(power) * maxPower;
        }
        power += feedforward;

//        if (Math.abs(c-t) > 10) {
//            double error = t-c;
//            power += 0.01 * Math.signum(error);
//        }

        if (Math.abs(power) > maxPower) {
            power = maxPower * Math.signum(power);
        }

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
