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
    private FeedForwardController controller;
    private VoltageSensor voltageSensor;
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
                double power = controller.calculate(shooterMotor.getVelocity(), targetVelocity);
                power *= (nominalVoltage / voltageSensor.getVoltage());
                shooterMotor.setPower(power);
                shooterMotor2.setPower(power);
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
        pitchServo.setPosition(MathUtil.clamp(pos, PITCH_SERVO_I, PITCH_SERVO_F));
    }

    public void setTargetVelocity(double t) {
        targetVelocity = t;
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }
    public double getPower() {return shooterMotor.getPower();}


    // useful trust
    public boolean atTarget(double threshold) {
        return Math.abs(shooterMotor.getVelocity()-targetVelocity) < threshold;
    }

    @Override
    public void start() {}
}
