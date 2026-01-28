package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

public class LimelightLocalizer extends Subsystem {
    public enum Pipeline {
        APRILTAG,
        ARTIFACT_DETECTION
    }
    private final Limelight3A limelight;
    private Pipeline pipeline;
    private double staleness = 0;
    private Pose currentPose = new Pose(72, 72);
    private double largestClusterX = 0;

    // TODO: add an extra servo, which adjusts pitch based on pipeline
    public LimelightLocalizer(HardwareMap hardwareMap) {
        currentPose = new Pose();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();
    }

    public void setPipeline(Pipeline pipeline) {
        this.pipeline = pipeline;
        if (pipeline == Pipeline.APRILTAG) {
            limelight.pipelineSwitch(3);
        } else if (pipeline == Pipeline.ARTIFACT_DETECTION) {
            limelight.pipelineSwitch(2);
        }
    }
    private double metersToInches(double meters) {
        return meters * 39.3701;
    }

    private Pose toPinpointPose(Pose3D llpose, double heading) {
        double x = 72 + (metersToInches(llpose.getPosition().y));
        double y = 72 - (metersToInches(llpose.getPosition().x));
        return new Pose(x, y, heading);
    }

    @Override
    public void update() {
        LLResult result = limelight.getLatestResult();
        staleness = result.getStaleness();
        if (result != null && result.isValid()) {
            if (pipeline == Pipeline.APRILTAG) {
                Pose3D botPose = result.getBotpose();
                double heading = MathUtil.normalizeAngle(Math.toRadians(result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES) - 270));
                currentPose = toPinpointPose(botPose, heading);
            }
            if (pipeline == Pipeline.ARTIFACT_DETECTION) {
                double[] pythonOutput = result.getPythonOutput();
                largestClusterX = pythonOutput[0];
            }
        }
    }

    public Pose getCurrentPose(Pose pinpointPose) {
        if (currentPose.getX() == 72 && currentPose.getY() == 72) {
            return pinpointPose;
        }
        return currentPose;
    }

    @Override
    public void start() {
        limelight.start();
    }
}