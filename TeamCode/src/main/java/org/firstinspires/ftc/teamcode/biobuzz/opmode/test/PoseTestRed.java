package org.firstinspires.ftc.teamcode.decode2026.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name="Pose Test Red")
public class PoseTestRed extends OpMode {
    private Follower follower;
    // TODO: set this accordingly
    private final Pose startPose = new Pose();

    @Override
    public void loop() {
        follower.update();
        telemetry.addLine(follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {

    }
}
