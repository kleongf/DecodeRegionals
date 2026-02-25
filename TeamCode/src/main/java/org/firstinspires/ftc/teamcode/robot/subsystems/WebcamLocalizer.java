package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;
import android.util.Size;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
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
    private ElapsedTime timer;
    private Servo light;
    private Position cameraPosition = new Position(DistanceUnit.MM,
            138, 119, 236, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -70, 0, 0);
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(PoseConstants.FIELD_WIDTH / 2d + webcamPose.getY(), PoseConstants.FIELD_WIDTH / 2d - webcamPose.getX(), webcamPose.getHeading());
    }
    public static AprilTagLibrary getDecodeTagLibraryAdjusted(){
        return new AprilTagLibrary.Builder()
                .addTag(20, "BlueTarget",
                        6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                .addTag(21, "Obelisk_GPP",
                        6.5, DistanceUnit.INCH)
                .addTag(22, "Obelisk_PGP",
                        6.5, DistanceUnit.INCH)
                .addTag(23, "Obelisk_PPG",
                        6.5, DistanceUnit.INCH)
                .addTag(24, "RedTarget",
                        6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
                .build();
    }

    public WebcamLocalizer(HardwareMap hardwareMap) {
        currentPose = new Pose();
        timer = new ElapsedTime();
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(0);

        aprilTag = new AprilTagProcessor.Builder()
                // red: off by -3 inches (so add 3)
                // blue: off by 3 inches (so subtract 3)
                // this is for y coord btw

                // theres no way its off by that much bruh
                .setLensIntrinsics(915.89533774, 916.57002166, 665.64617643, 423.48045066)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(getDecodeTagLibraryAdjusted())
                .setCameraPose(cameraPosition, cameraOrientation)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 800));

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        // builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // TODO: just for now so we can check feed
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    @Override
    public void update() {
        // PROBLEM: we can see two at once. which one do we trust?
        // fortunately, when we relocalize based on the distance constraint, we are unable to see both

        // so this is a non-issue
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    if (detection.metadata.name.contains("BlueTarget")) {
                        Pose ppPose = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                        currentPose = ppPose;
                        isGoodDetection = true;
                    }
                    if (detection.metadata.name.contains("RedTarget")) {
                        Pose ppPose = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                        currentPose = ppPose;
                        isGoodDetection = true;
                    }
                }
            }
        }
        if (currentDetections.isEmpty()) {
            isGoodDetection = false;
        }

        if (timer.seconds() > 1) {
            light.setPosition(0);
        }
    }

    @Override
    public void start() {}

    public Pose getCurrentPose() {return currentPose;}
    public boolean getIsGoodDetection() {return isGoodDetection;}

    public void flashLED() {
        light.setPosition(1.0);
        timer.reset();
    }
}
