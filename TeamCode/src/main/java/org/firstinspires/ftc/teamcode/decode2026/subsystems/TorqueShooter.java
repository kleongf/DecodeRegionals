package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.decode2026.constants.TorqueShooterConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

public class TorqueShooter extends Subsystem {
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
    private final VoltageSensor voltageSensor;
    private final double R = 9.2 / 0.144; // motor resistance 1.3 ohms ish
    private final double kOmega = 12 / 2800d; // back emf, volts ticks^-1 s^-1
    public TorqueShooter(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        latchServo = hardwareMap.get(Servo.class, "latchServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
    }
    // wanted torque = kP * velocity error, ik it sounds weird but trust even tho they not same units
    private double calculateVoltageOutput(double wantedTorque, double currentVelocity) {
        return R * wantedTorque + kOmega * currentVelocity;
    }

    @Override
    public void reset() {
        wantedMode = Mode.SHOOTER_OFF;
        closeLatch();
    }

    @Override
    public void start() {
        wantedMode = Mode.SHOOTER_ON;
    }

    @Override
    public void update() {
        currentVelocity = shooterMotor.getVelocity();

        switch (wantedMode) {
            case SHOOTER_ON:
                double error = wantedVelocity - currentVelocity;
                double wantedTorque = TorqueShooterConstants.kP * error;
                double power = calculateVoltageOutput(wantedTorque, currentVelocity) / TorqueShooterConstants.nominalVoltage;
                power = power + 
                        TorqueShooterConstants.kS * Math.signum(error) + 
                        TorqueShooterConstants.kV * wantedVelocity + 
                        TorqueShooterConstants.kA * wantedAcceleration;

                if (TorqueShooterConstants.useVoltageCompensation) {
                    power *= (TorqueShooterConstants.nominalVoltage / voltageSensor.getVoltage());
                }
                
                shooterMotor.setPower(power);
                shooterMotor2.setPower(power);

                double ticksPerRadian = (TorqueShooterConstants.PITCH_SERVO_F-TorqueShooterConstants.PITCH_SERVO_I)/(TorqueShooterConstants.PITCH_F-TorqueShooterConstants.PITCH_I);
                double adjustedAngle = wantedPitch - TorqueShooterConstants.PITCH_I;
                double pos = TorqueShooterConstants.PITCH_SERVO_MIN + adjustedAngle * ticksPerRadian;
                if (!Double.isNaN(pos)) {
                    double lower = Math.min(TorqueShooterConstants.PITCH_SERVO_I, TorqueShooterConstants.PITCH_SERVO_F);
                    double upper = Math.max(TorqueShooterConstants.PITCH_SERVO_I, TorqueShooterConstants.PITCH_SERVO_F);
                    pitchServo.setPosition(MathUtil.clamp(pos, lower, upper));
                }
                break;
            case SHOOTER_OFF:
                break;
        }
    }
    public void openLatch() {
        latchServo.setPosition(TorqueShooterConstants.LATCH_OPEN);
    }
    public void closeLatch() {
        latchServo.setPosition(TorqueShooterConstants.LATCH_CLOSED);
    }

    public boolean atTarget(double threshold) {
        return Math.abs(currentVelocity - wantedVelocity) < threshold;
    }
}
