package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode2026.constants.CameraLocalizerConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
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
    public Alliance alliance;
    private final AprilTagProcessor aprilTag;
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(FieldConstants.FIELD_WIDTH / 2d + webcamPose.getY(), FieldConstants.FIELD_WIDTH / 2d - webcamPose.getX(), webcamPose.getHeading());
    }
    public CameraLocalizer(HardwareMap hardwareMap) {
        alliance = Alliance.BLUE;

        Position cameraPosition = new Position(DistanceUnit.MM,
                0, 0, 272, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -70, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(CameraLocalizerConstants.fxLeft, CameraLocalizerConstants.fyLeft, CameraLocalizerConstants.cxLeft, CameraLocalizerConstants.cyLeft)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
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
                        Pose cameraFieldConverted = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                        double cameraFieldXConverted = cameraFieldConverted.getX();
                        double cameraFieldYConverted = cameraFieldConverted.getY();
                        double cameraFieldHeadingConverted = cameraFieldConverted.getHeading();

                        // translate backward by the distance to center along the camera's global heading line
                        double robotX = cameraFieldXConverted - (MathUtil.mmToIn(153) * Math.cos(cameraFieldHeadingConverted));
                        double robotY = cameraFieldYConverted - (MathUtil.mmToIn(153) * Math.sin(cameraFieldHeadingConverted));

                        // calculate robot heading based on turret rotation
                        double turretAngle = MathUtil.angleWrap(currentAngle);
                        double robotHeading = cameraFieldHeadingConverted - turretAngle;
                        // yayyyyy uwu kitty cat meow
                        bestPose = new Pose(robotX, robotY, robotHeading);
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