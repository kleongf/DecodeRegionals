package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@Config
@TeleOp(name="limelight pose test blue standard start pose")
public class LimelightPoseTestBlueStandard extends OpMode {
    // this assumes the limelight is like 3 inches right from center of robot, up like 9 inches, and angled at 45
    private Follower follower;
    private Intake intake;
    private final Pose startPose = PoseConstants.BLUE_STANDARD_START_POSE;


    @Override
    public void loop() {
        follower.update();
        intake.update();
        telemetry.addLine(follower.getPose().toString());
        // 14.7, 59.5
                // 32. 135.8

        telemetry.update();
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new Intake(hardwareMap);
        intake.state = Intake.IntakeState.INTAKE_FAST;
    }

    @Override
    public void start() {

    }
}
