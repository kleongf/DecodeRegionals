package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;

@Config
@TeleOp(name="Flywheel Feedforward Tuner")
public class FeedForwardTuner extends OpMode {
    private FeedForwardController controller;
    public static double kV = 0;
    public static double kS = 0;
    public static double kP = 0;

    public static int target = 0;

    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new FeedForwardController(0, 0, 0);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // TODO: max velocity is 2800 ticks/s
    }

    @Override
    public void loop() {
        controller.setCoefficients(kV, kS, kP);
        double currentVel = shooterMotor.getVelocity();
        double power = controller.calculate(currentVel, target);
        shooterMotor.setPower(power);
        shooterMotor2.setPower(power);

        telemetry.addData("power", power);
        telemetry.addData("current velocity", currentVel);
        telemetry.addData("target", target);
        telemetry.update();
    }

}
