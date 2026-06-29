package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode2026.constants.CameraLocalizerConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class CameraLocalizer extends Subsystem {
    public enum Mode {
        CAMERA_ON,
        CAMERA_OFF
    }
    public Mode wantedMode;
    public Pose currentPose;
    public Alliance alliance;
    public boolean isGoodDetection;
    private final AprilTagProcessor aprilTagLeft;
    private final AprilTagProcessor aprilTagRight;
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(FieldConstants.FIELD_WIDTH / 2d + webcamPose.getY(), FieldConstants.FIELD_WIDTH / 2d - webcamPose.getX(), webcamPose.getHeading());
    }
    public CameraLocalizer(HardwareMap hardwareMap) {
        alliance = Alliance.BLUE;
        currentPose = new Pose();

        aprilTagLeft = new AprilTagProcessor.Builder()
                .setLensIntrinsics(CameraLocalizerConstants.fxLeft, CameraLocalizerConstants.fyLeft, CameraLocalizerConstants.cxLeft, CameraLocalizerConstants.cyLeft)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(CameraLocalizerConstants.cameraPositionLeft, CameraLocalizerConstants.cameraOrientationLeft)
                .build();

        aprilTagRight = new AprilTagProcessor.Builder()
                .setLensIntrinsics(CameraLocalizerConstants.fxRight, CameraLocalizerConstants.fyRight, CameraLocalizerConstants.cxRight, CameraLocalizerConstants.cyRight)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(CameraLocalizerConstants.cameraPositionRight, CameraLocalizerConstants.cameraOrientationRight)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(CameraLocalizerConstants.cameraResolutionRight);

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // todo: comment out because wastes cpu
        // builder.enableLiveView(true);
        builder.addProcessors(aprilTagLeft, aprilTagRight);
        builder.setLiveViewContainerId(0);

        builder.build();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void reset() {
        wantedMode = Mode.CAMERA_OFF;
    }

    @Override
    public void start() {
        wantedMode = Mode.CAMERA_ON;
    }

    @Override
    public void update() {
        switch (wantedMode) {
            // if multiple detections, use the closest one because it will be the most accurate
            // apparently aprilTag.getFreshDetections() is more efficient
            case CAMERA_ON:
                List<AprilTagDetection> currentDetections;
                if (alliance == Alliance.BLUE) {
                    currentDetections = aprilTagRight.getDetections();
                } else {
                    currentDetections = aprilTagLeft.getDetections();
                }
                double bestDistance = Integer.MAX_VALUE;
                Pose bestPose = null;

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.metadata.name.contains("BlueTarget") || detection.metadata.name.contains("RedTarget")) {
                            isGoodDetection = true;
                            double distance = detection.ftcPose.range;
                            if (distance < bestDistance) {
                                bestDistance = distance;
                                bestPose = toPinpointPose(new Pose(
                                        detection.robotPose.getPosition().x,
                                        detection.robotPose.getPosition().y,
                                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                            }
                        }
                    }
                }

                if (bestPose != null) {
                    currentPose = bestPose;
                } else {
                    isGoodDetection = false;
                }
                break;
            case CAMERA_OFF:
                isGoodDetection = false;
                break;
        }
    }
}
