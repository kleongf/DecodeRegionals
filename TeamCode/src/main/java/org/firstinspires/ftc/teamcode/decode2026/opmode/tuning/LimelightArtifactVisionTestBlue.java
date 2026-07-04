package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Point;

import java.util.List;
@TeleOp(name="Limelight Artifact Vision Test Blue Standard Start", group="?")
public class LimelightArtifactVisionTestBlue extends OpMode {
    private ArtifactVision artifactVision;
    private Follower follower;
    @Override
    public void init() {
        artifactVision = new ArtifactVision(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FieldConstants.BLUE_STANDARD_START_POSE);
    }

    @Override
    public void start() {
        artifactVision.start();
    }

    @Override
    public void loop() {
        Pose robotPose = follower.getPose();
        telemetry.addLine("Camera Points");
        List<double[]> cameraPoints = artifactVision.getBlobs();
        telemetry.addData("Camera Points", cameraPoints);
        List<Point> worldPoints = artifactVision.getArtifactWorldPoints(robotPose);
        telemetry.addData("World Points", worldPoints);
        double bestY = artifactVision.findBestYPosition(robotPose, 8, 48);
        telemetry.addData("Best Y", bestY);

        follower.update();
        artifactVision.update();
    }
}
