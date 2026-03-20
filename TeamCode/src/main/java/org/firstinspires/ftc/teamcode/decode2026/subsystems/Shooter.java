package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.CachedMotor;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

public class Shooter extends Subsystem {
    public enum Mode {
        SHOOTER_ON,
        SHOOTER_OFF
    }
    public Mode wantedMode;
    public double wantedVelocity;
    public double currentVelocity;
    public double wantedPitch;
    public double wantedAcceleration;
    private final Servo latchServo;
    private final Servo pitchServo;
    private final DcMotorEx shooterMotor;
    private final DcMotorEx shooterMotor2;
    private final CachedMotor shooterMotorCached;
    private final CachedMotor shooterMotor2Cached;
    private final FeedForwardController controller;
    private final VoltageSensor voltageSensor;
    private double kA = ShooterConstants.kA;
    public Shooter(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorCached = new CachedMotor(shooterMotor).setCachingTolerance(ShooterConstants.cachingThreshold);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2Cached = new CachedMotor(shooterMotor2).setCachingTolerance(ShooterConstants.cachingThreshold);

        latchServo = hardwareMap.get(Servo.class, "latchServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        controller = new FeedForwardController(ShooterConstants.kV, ShooterConstants.kS, ShooterConstants.kP);
    }

    @Override
    public void reset() {
        wantedMode = Mode.SHOOTER_OFF;
    }

    @Override
    public void start() {
        wantedMode = Mode.SHOOTER_ON;
    }

    @Override
    public void update() {
        currentVelocity = -shooterMotorCached.getVelocity();

        switch (wantedMode) {
            case SHOOTER_ON:
                // TODO: remember to set velocity before update() which i think we do.
                double power = controller.calculate(currentVelocity, wantedVelocity);
                power += kA * wantedAcceleration;

                if (ShooterConstants.useVoltageCompensation) {
                    power *= (ShooterConstants.nominalVoltage / voltageSensor.getVoltage());
                }
                shooterMotorCached.setPower(power);
                shooterMotor2Cached.setPower(power);

                double ticksPerRadian = (ShooterConstants.PITCH_SERVO_F-ShooterConstants.PITCH_SERVO_I)/(ShooterConstants.PITCH_F-ShooterConstants.PITCH_I);
                double adjustedAngle = wantedPitch - ShooterConstants.PITCH_I;
                double pos = ShooterConstants.PITCH_SERVO_MIN + adjustedAngle * ticksPerRadian;
                if (!Double.isNaN(pos)) {
                    double lower = Math.min(ShooterConstants.PITCH_SERVO_I, ShooterConstants.PITCH_SERVO_F);
                    double upper = Math.max(ShooterConstants.PITCH_SERVO_I, ShooterConstants.PITCH_SERVO_F);
                    pitchServo.setPosition(MathUtil.clamp(pos, lower, upper));
                }
                break;
            case SHOOTER_OFF:
                break;
        }
    }
    public void openLatch() {
        latchServo.setPosition(ShooterConstants.LATCH_OPEN);
    }
    public void closeLatch() {
        latchServo.setPosition(ShooterConstants.LATCH_CLOSED);
    }

    public boolean atTarget(double threshold) {
        return Math.abs(currentVelocity - wantedVelocity) < threshold;
    }

    public void setkA(double ka) {
        kA = ka;
    }

    public void setkPkV(double kp, double kv) {
        controller.setCoefficients(kv, ShooterConstants.kS, kp);
    }
}

