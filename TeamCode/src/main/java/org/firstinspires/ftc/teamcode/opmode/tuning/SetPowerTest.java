package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;

@Config
@TeleOp(name="SetPower test")
public class SetPowerTest extends OpMode {
    public DcMotorEx intakeMotor;
    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;
    public static double shooterPower = 0;
    public static double intakePower = 0;

    private FeedForwardController controller;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        shooterMotor.setPower(shooterPower);
        shooterMotor2.setPower(shooterPower);

        telemetry.addData("shooter power: ", shooterPower);
        telemetry.addData("shooter velo: ", shooterMotor.getVelocity());


        intakeMotor.setPower(intakePower);


    }
}
