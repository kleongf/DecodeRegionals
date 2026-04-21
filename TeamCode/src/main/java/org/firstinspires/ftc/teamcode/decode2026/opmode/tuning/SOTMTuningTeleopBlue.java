package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode2026.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.decode2026.opmode.teleop.MainTeleop;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;

@Config
@TeleOp(name="SOTM teleop tuning BLUE", group="?")
public class SOTMTuningTeleopBlue extends OpMode {
    private MainTeleop teleop;
    private final Pose startPose = FieldConstants.BLUE_STANDARD_START_POSE;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new MainTeleop(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        teleop.loop();
        // graphing these could make it a lot easier to tune.
        telemetry.addData("robot velocity magnitude", teleop.drivetrain.getVelocity().getMagnitude());
        telemetry.addData("turret feedforward power", teleop.robot.turret.wantedAngularVelocity * TurretConstants.kV);
        telemetry.addData("current turret pos", teleop.robot.turret.currentPositionTicks);
        telemetry.addData("target turret pos", teleop.robot.turret.wantedAngle * TurretConstants.ticksPerRadian);
        telemetry.addData("current flywheel speed", teleop.robot.shooter.currentVelocity);
        telemetry.addData("target flywheel speed", teleop.robot.shooter.wantedVelocity);
        telemetry.addData("flywheel feedforward power", teleop.robot.shooter.wantedAcceleration * ShooterConstants.kA);
        telemetry.update();
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
