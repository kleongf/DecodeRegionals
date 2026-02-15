package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
@Config
@TeleOp(name="Shooter Tuner")
public class TurretFollowTest extends OpMode {
    private Turret turret;
    public static double shooterSpeed;
    public static double shooterPitch;
    public static double offset = 0;
    private Follower follower;
    private Intake intake;
    private SOTM sotm2;
    private Shooter shooter;
    private final Pose startPose = PoseConstants.BLUE_STANDARD_START_POSE;
    // i don't think angle matters here in the sotm calculation
    private final Pose goalPose = new Pose(0, 141, Math.toRadians(45));
    @Override
    public void loop() {

        shooter.setTargetVelocity(shooterSpeed);
        shooter.setShooterPitch(Math.toRadians(shooterPitch));
        // for tuning we are gonna explicitly offset
        // sotm2.offsetFactor = offset; // in case we get off
        double[] target = sotm2.calculateAzimuthThetaVelocityFRCBetter(follower.getPose(), new Vector(), follower.getAngularVelocity());
        // set azimuth
        turret.setTarget(target[0]+offset);
        // turret.setFeedforward(target[3]);


        double dx = goalPose.getX()-follower.getPose().getX();
        double dy = goalPose.getY()-follower.getPose().getY();

        double dist = Math.hypot(dx, dy);

        follower.update();
        turret.update();
        shooter.update();
        intake.update();

        telemetry.addData("distance", dist);
        telemetry.addData("shooter speed", shooter.getCurrentVelocity());
        telemetry.addData("shooter motor 2 speed", shooter.getCurrentVelocity2());
        telemetry.addData("encoder velocity diff", shooter.getVelocityDiff());
        // telemetry.addData("shooter power", shooter.getPower());
        // telemetry.addData("turret at target? 1.5% error", turret.atTarget(20));
        telemetry.update();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        shooter = new Shooter(hardwareMap);
        shooter.state = Shooter.ShooterState.SHOOTER_ON;
        intake = new Intake(hardwareMap);
        intake.state = Intake.IntakeState.INTAKE_FAST;
        sotm2 = new SOTM(goalPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        //follower.startTeleopDrive();
    }
}
