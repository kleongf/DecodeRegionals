package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration((-33 + (-30)) / 2.0)
            .lateralZeroPowerAcceleration(((-50) + (-62) + (-58)) / 3.0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.10, 0.10, 0.001))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.000)
            .holdPointHeadingScaling(0.35)
            .holdPointTranslationalScaling(0.35)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.14, 0, 0.008, 0.0))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.2, 0, 0.012, 0.0)
            )
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.08, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0, 0.0004, 0.6, 0)
            )
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0, 0.0008, 0.6, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("front_left_drive")
            .leftRearMotorName("back_left_drive")
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(87.67)
            .yVelocity(58.98)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true)
            .nominalVoltage(12.0);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.30709)
            .strafePodX(-3.77953-0.04)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.975,
            0,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

