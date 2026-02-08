package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

public class Shooter extends Subsystem {
    public enum ShooterState {
        SHOOTER_ON,
        SHOOTER_OFF
    }

    public ShooterState state = ShooterState.SHOOTER_OFF;
    private double targetVelocity = 0;
    private Servo latchServo;
    private Servo pitchServo;
    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;
    private double epsilon = 100;
    private double shootingEpsilon = 60;
    private FeedForwardController controller;
    private VoltageSensor voltageSensor;
    private boolean isShooting = false;
    private double nominalVoltage = 12.4; // i would say that it's usually 12.4 when tuning.
    public Shooter(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        latchServo = hardwareMap.get(Servo.class, "latchServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        controller = new FeedForwardController((1.0/2180), 0, 0.003);
    }
    @Override
    public void update() {
        switch (state) {
            case SHOOTER_ON:
                // || (shooterMotor.getVelocity() < targetVelocity && (Math.abs(shooterMotor.getVelocity()-targetVelocity)) > shootingEpsilon && isShooting)
                if (shooterMotor.getVelocity()+shootingEpsilon < targetVelocity) {
                    shooterMotor.setPower(1);
                    shooterMotor2.setPower(1);
                } else if (shooterMotor.getVelocity()-shootingEpsilon > targetVelocity) {
                    shooterMotor.setPower(0);
                    shooterMotor2.setPower(0);
                } else {
                    double power = controller.calculate(shooterMotor.getVelocity(), targetVelocity);
                    power *= (nominalVoltage / voltageSensor.getVoltage());
                    shooterMotor.setPower(power);
                    shooterMotor2.setPower(power);
                }

                // if (shooterMotor.getVelocity())

//                if (shooterMotor.getVelocity() < targetVelocity && (Math.abs(shooterMotor.getVelocity()-targetVelocity)) > epsilon) {
//                    shooterMotor.setPower(1);
//                    shooterMotor2.setPower(1);
//                } else {
//                    shooterMotor.setPower(0);
//                    shooterMotor2.setPower(0);
//
//                }
//                if (isShooting) {
//                    if (shooterMotor.getVelocity() < targetVelocity) {
//                        shooterMotor.setPower(1);
//                        shooterMotor2.setPower(1);
//                    }
//                }
//                double power = controller.calculate(shooterMotor.getVelocity(), targetVelocity);
//                power *= (nominalVoltage / voltageSensor.getVoltage());
//
//                // new idea that i saw online:
//                // this controller is good at getting up to speed but not as good at maintining when shooting
//                // TODO: if it ever becomes a problem, switch to bang bang controller when shooting
//                shooterMotor.setPower(power);
//                shooterMotor2.setPower(power);
        }
    }

    public void openLatch() {
        latchServo.setPosition(LATCH_OPEN);
    }

    public void closeLatch() {
        latchServo.setPosition(LATCH_CLOSED);
    }
    // note: in radians
    public void setShooterPitch(double angle) {
        double ticksPerRadian = (PITCH_SERVO_F-PITCH_SERVO_I)/(PITCH_F-PITCH_I);
        double adjustedAngle = angle - PITCH_I;
        double pos = PITCH_SERVO_MIN + adjustedAngle * ticksPerRadian;
        if (!Double.isNaN(pos)) {
            double lower = Math.min(PITCH_SERVO_I, PITCH_SERVO_F);
            double upper = Math.max(PITCH_SERVO_I, PITCH_SERVO_F);
            pitchServo.setPosition(MathUtil.clamp(pos, lower, upper));
        }
    }

    public void setTargetVelocity(double t) {
        targetVelocity = t;
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }
    public double getCurrentVelocity2() {return shooterMotor2.getVelocity();}
    public double getVelocityDiff() {return Math.abs(shooterMotor.getVelocity() - shooterMotor2.getVelocity());}
    public double getPower() {return shooterMotor.getPower();}
    public void setIsShooting(boolean x) {isShooting = x;}


    // useful trust
    public boolean atTarget(double threshold) {
        return Math.abs(shooterMotor.getVelocity()-targetVelocity) < threshold;
    }

    @Override
    public void start() {}
}
