package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode2026.constants.CameraLocalizerConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TurretCameraLocalizer extends Subsystem {
    public enum Mode {
        CAMERA_ON,
        CAMERA_OFF
    }
    public Mode wantedMode;
    public Alliance alliance;
    private final AprilTagProcessor aprilTag;
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(FieldConstants.FIELD_WIDTH / 2d + webcamPose.getY(), FieldConstants.FIELD_WIDTH / 2d - webcamPose.getX(), webcamPose.getHeading());
    }
    public TurretCameraLocalizer(HardwareMap hardwareMap) {
        alliance = Alliance.BLUE;

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(CameraLocalizerConstants.fxLeft, CameraLocalizerConstants.fyLeft, CameraLocalizerConstants.cxLeft, CameraLocalizerConstants.cyLeft)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(CameraLocalizerConstants.cameraPositionLeft, CameraLocalizerConstants.cameraOrientationLeft)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(CameraLocalizerConstants.cameraResolutionLeft);

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.addProcessor(aprilTag);
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

    public Pose getPoseFromApriltag(double currentAngle) {
        if (wantedMode == Mode.CAMERA_OFF) {
            return new Pose();
        }
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double bestDistance = Integer.MAX_VALUE;
        Pose bestPose = new Pose();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("BlueTarget") || detection.metadata.name.contains("RedTarget")) {
                    double distance = detection.ftcPose.range;
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        // note: for some reason, robot always thinks its x position is closer to the tag than it should be, but y is good? ask gpt
                        double cameraFieldX = detection.robotPose.getPosition().x;
                        double cameraFieldY = detection.robotPose.getPosition().y;
                        double cameraFieldHeading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                        // translate backward by the distance to center along the camera's global heading line
                        double robotX = cameraFieldX - (MathUtil.mmToIn(153) * Math.cos(cameraFieldHeading));
                        double robotY = cameraFieldY - (MathUtil.mmToIn(153) * Math.sin(cameraFieldHeading));

                        // calculate robot heading based on turret rotation
                        double turretAngle = MathUtil.normalizeAngle(currentAngle) - Math.PI;
                        double robotHeading = cameraFieldHeading - turretAngle;

                        bestPose = toPinpointPose(new Pose(robotX, robotY, robotHeading));
                    }
                }
            }
        }
        return bestPose;
    }

    @Override
    public void update() {
        super.update();
    }
}
