package org.firstinspires.ftc.teamcode.decode2026.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
@Disabled
@TeleOp(name="Blue Teleop Tuning", group="!")
public class TuningTeleopBlue extends OpMode {
    private TuningTeleop teleop;
    private final Pose startPose = FieldConstants.BLUE_STANDARD_START_POSE;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new TuningTeleop(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        teleop.loop();
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

/*
50, -1
80, -2
134, -3
154, -3
 */

