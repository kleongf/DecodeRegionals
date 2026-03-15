package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode2026.constants.CameraLocalizerConstants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraLocalizer extends Subsystem {
    public enum Mode {
        CAMERA_ON,
        CAMERA_OFF
    }
    public Mode wantedMode;
    public Pose currentPose;
    public boolean isGoodDetection;
    private final AprilTagProcessor aprilTag;
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(PoseConstants.FIELD_WIDTH / 2d + webcamPose.getY(), PoseConstants.FIELD_WIDTH / 2d - webcamPose.getX(), webcamPose.getHeading());
    }
    public CameraLocalizer(HardwareMap hardwareMap) {
        currentPose = new Pose();

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(CameraLocalizerConstants.fx, CameraLocalizerConstants.fy, CameraLocalizerConstants.cx, CameraLocalizerConstants.cy)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(CameraLocalizerConstants.cameraPosition, CameraLocalizerConstants.cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(CameraLocalizerConstants.cameraResolution);

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);
        builder.addProcessor(aprilTag);

        builder.build();
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
            case CAMERA_ON:
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (!detection.metadata.name.contains("Obelisk")) {
                            if (detection.metadata.name.contains("BlueTarget")) {
                                currentPose = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                                isGoodDetection = true;
                            }
                            if (detection.metadata.name.contains("RedTarget")) {
                                currentPose = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                                isGoodDetection = true;
                            }
                        }
                    }
                }
                if (currentDetections.isEmpty()) {
                    isGoodDetection = false;
                }
                break;
            case CAMERA_OFF:
                isGoodDetection = false;
                break;
        }
    }
}
