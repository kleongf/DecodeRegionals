package org.firstinspires.ftc.teamcode.opmode.tuning;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "webcam localization test new camera", group = "Concept")
public class WebcamLocalizationTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private Follower follower;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public static AprilTagLibrary getDecodeTagLibraryAdjusted(){
        return new AprilTagLibrary.Builder()
                .addTag(20, "BlueTarget",
                        6.5, new VectorF(-58.3727f + 1.5f, -55.6425f + 1.5f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                .addTag(21, "Obelisk_GPP",
                        6.5, DistanceUnit.INCH)
                .addTag(22, "Obelisk_PGP",
                        6.5, DistanceUnit.INCH)
                .addTag(23, "Obelisk_PPG",
                        6.5, DistanceUnit.INCH)
                .addTag(24, "RedTarget",
                        6.5, new VectorF(-58.3727f + 1.5f, 55.6425f - 1.5f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
                .build();
    }

    @Override
    public void runOpMode() {

        initAprilTag();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstants.BLUE_STANDARD_START_POSE);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                follower.update();

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU. NO lol
                // sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        /**
         * Variables to store the position and orientation of the camera on the robot. Setting these
         * values requires a definition of the axes of the camera and robot:
         *
         * Camera axes:
         * Origin location: Center of the lens
         * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
         *
         * Robot axes (this is typical, but you can define this however you want):
         * Origin location: Center of the robot at field height
         * Axes orientation: +x right, +y forward, +z upward
         *
         * Position:
         * If all values are zero (no translation), that implies the camera is at the center of the
         * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
         * inches above the ground - you would need to set the position to (-5, 7, 12).
         *
         * Orientation:
         * If all values are zero (no rotation), that implies the camera is pointing straight up. In
         * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
         * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
         * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
         * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
         */
        // TODO: GET THIS POSITION RIGHT
        // -122, 142, 230, 0); // this camera is on the other side: on the right side
        Position cameraPosition = new Position(DistanceUnit.MM,
                138, 119, 236, 0);
        // straight up is zero, so i guess 20 deg up would be -70
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -70, 0, 0);

        /*
            private Matrix K = new Matrix(new double[][] {
            {214.1037056, 0, 313},
            {0, 212.72822576, 254.488},
            {0, 0, 1}
    });
         */

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(920.46723598, 918.07391093, 653.71790268, 406.18310197)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(getDecodeTagLibraryAdjusted())
                .setCameraPose(cameraPosition, cameraOrientation)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);

        // aprilTag.setDecimation(2);

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 800));

        // Set the camera (webcam vs. built-in RC phone camera).
        // TODO: set to webcam 2 when we get the connector thingy
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private Pose toPinpointPose(Pose webcamPose) {
        return new Pose(70.5 + webcamPose.getY(), 70.5 - webcamPose.getX(), webcamPose.getHeading());
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // TODO: use the degree output to power the pid
                // say when we have the trigger pressed down, we turn on the turret pid thingy
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    Pose ppPose = toPinpointPose(new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                    telemetry.addLine("Pinpoint Pose: " + ppPose.toString());
                    telemetry.addLine("Follower pose:" + follower.getPose());
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class
