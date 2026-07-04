package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
@TeleOp(name="Red Teleop Tuning Offset", group="!")
public class TuningTeleopRed extends OpMode {
    private TuningTeleop teleop;
    private final Pose startPose = FieldConstants.RED_STANDARD_START_POSE;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new TuningTeleop(startPose, Alliance.RED, hardwareMap, telemetry, gamepad1, gamepad2);
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
134: -1
154: -1
90: 0
otherwise: 0
 */
