package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;

@Config
@TeleOp(name="limelight pose test red standard start pose")
public class LimelightPoseTestRedStandard extends OpMode {
    private Follower follower;
    private Intake intake;
    private final Pose startPose = FieldConstants.RED_STANDARD_START_POSE;


    @Override
    public void loop() {
        follower.update();
        intake.update();
        telemetry.addLine(follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {

    }
}
