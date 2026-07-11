package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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
import org.firstinspires.ftc.teamcode.decode2026.constants.CameraLocalizerConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Field;
import java.util.List;

@TeleOp(name = "turret webcam localization test", group = "?")
public class TurretWebcamLocalizationTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private Follower follower;
    private Turret turret;
    private SOTMUtil sotm;
    private VisionPortal visionPortal;
    private final double CAMERA_DISTANCE_TO_CENTER = MathUtil.mmToIn(153); // idk rn


    @Override
    public void runOpMode() {
        initAprilTag();
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FieldConstants.BLUE_STANDARD_START_POSE);
        turret = new Turret(hardwareMap);
        turret.reset();
        turret.start();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                ShootingConstants.ShooterOutputs shooterOutputs = sotm.calculateShooterOutputsTuning(follower.getPose(), new Vector(), new Vector(), 0, 0.02, Alliance.BLUE);
                turret.wantedAngle = shooterOutputs.turretAngle;
                turret.wantedAngularVelocity = 0;

                follower.update();
                turret.update();

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
        // TODO: GET THIS POSITION RIGHT (only z and pitch matter)
        // actually left cam is better calibrated so we gonna use it

        Position cameraPosition = new Position(DistanceUnit.MM,
                0, 0, 272, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -70, 0, 0);


        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // i think it may be fx focal length
                .setLensIntrinsics(CameraLocalizerConstants.fxLeft, CameraLocalizerConstants.fyLeft, CameraLocalizerConstants.cxLeft, CameraLocalizerConstants.cyLeft)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 800));

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()

    private Pose toPinpointPose(Pose webcamPose) {
        return new Pose(71 + webcamPose.getY(), 71 - webcamPose.getX(), webcamPose.getHeading());
    }
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    // get pose from camera

                    // note: for some reason, robot always thinks its x position is closer to the tag than it should be, but y is good? ask gpt
                    double cameraFieldX = detection.robotPose.getPosition().x;
                    double cameraFieldY = detection.robotPose.getPosition().y;
                    double cameraFieldHeading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                    // translate backward by the distance to center along the camera's global heading line
                    double robotX = cameraFieldX - (CAMERA_DISTANCE_TO_CENTER * Math.cos(cameraFieldHeading));
                    double robotY = cameraFieldY - (CAMERA_DISTANCE_TO_CENTER * Math.sin(cameraFieldHeading));

                    // calculate robot heading based on turret rotation
                    double turretAngle = MathUtil.angleWrap(turret.currentAngle);
                    double robotHeading = cameraFieldHeading - turretAngle;
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    Pose ppPose = toPinpointPose(new Pose(robotX, robotY, robotHeading));
                    Pose cameraFieldPose = toPinpointPose(new Pose(cameraFieldX, cameraFieldY, cameraFieldHeading));
                    telemetry.addData("Turret angle", turretAngle);
                    telemetry.addLine("Pinpoint Pose: " + ppPose);
                    telemetry.addLine("Camera field pose: " + cameraFieldPose);
                }
            }
        }
        telemetry.addLine("Follower pose:" + follower.getPose());
    }   // end method telemetryAprilTag()

}   // end class

