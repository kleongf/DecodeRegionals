package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Size;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class WebcamLocalizer extends Subsystem {
    // (0, 0) -> (72, 72)
    // pp x is cam y
    // pp y is cam -x

    // ok this camera position is also probably wrong

    // (72+y), (72-x)
    private Pose currentPose;
    private boolean isGoodDetection = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.MM,
            -122, -230, 142, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(72 + webcamPose.getY(), 72 - webcamPose.getX(), webcamPose.getHeading());
    }

    public WebcamLocalizer(HardwareMap hardwareMap) {
        currentPose = new Pose();
        aprilTag = new AprilTagProcessor.Builder()
                // theres no way its off by that much bruh
                .setLensIntrinsics(214.1037056, 212.72822576, 320, 240)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(640, 480));

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    @Override
    public void update() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    Pose ppPose = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                    currentPose = ppPose;
                    isGoodDetection = true;
                }
            }
        }
        if (currentDetections.isEmpty()) {
            isGoodDetection = false;
        }
    }

    @Override
    public void start() {}

    public Pose getCurrentPose() {return currentPose;}
    public boolean getIsGoodDetection() {return isGoodDetection;}
}
