package org.firstinspires.ftc.teamcode.opmode.comp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;

@Config
@TeleOp(name="Robot Test #2 COMP: individual motor test", group = "!")
public class RobotTest2 extends OpMode {
    public Servo leftLatch;
    public Servo liftServo;
    public Servo pitchServo;
    public DcMotorEx intakeMotor;
    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;
    public static double latchTarget = RobotConstants.LATCH_CLOSED;
    public static double shooterPower1 = 0;
    public static double shooterPower2 = 0;
    public static double intakePower = 0;
    public static double liftTarget = 0;
    public static double pitchTarget = RobotConstants.PITCH_SERVO_MIN;

    private FeedForwardController controller;

    @Override
    public void init() {
        leftLatch = hardwareMap.get(Servo.class, "latchServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        liftServo = hardwareMap.get(Servo.class, "hang1");

        // open 0.3 (not hung), close 0.54 (hung)


        leftLatch.setDirection(Servo.Direction.FORWARD);
        pitchServo.setDirection(Servo.Direction.FORWARD);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.1) {
            shooterMotor.setPower(1);
        } else {
            shooterMotor.setPower(0);
        }

        if (gamepad1.right_trigger > 0.1) {
            shooterMotor2.setPower(1);
        } else {
            shooterMotor2.setPower(0);
        }
//        shooterMotor.setPower(shooterPower1);
//        shooterMotor2.setPower(shooterPower2);

        intakeMotor.setPower(intakePower);

        leftLatch.setPosition(latchTarget);
        pitchServo.setPosition(pitchTarget);

        liftServo.setPosition(liftTarget);
    }
}
