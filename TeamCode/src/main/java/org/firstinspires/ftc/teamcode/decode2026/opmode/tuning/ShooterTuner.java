package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.TorqueShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;

@Disabled
@Config
@TeleOp(name="Shooter Tuner")
public class ShooterTuner extends OpMode {
    private Turret turret;
    public static double shooterSpeed;
    public static double shooterPitchDegrees; // degrees for tuning
    public static double offsetDegrees = 0; // degrees for tuning
    private Follower follower;
    private Intake intake;
    private SOTMUtil sotm;
    private TorqueShooter shooter;
    private final Pose startPose = FieldConstants.BLUE_STANDARD_START_POSE;
    private final Pose goalPose = FieldConstants.BLUE_GOAL_POSE;

    @Override
    public void loop() {
        shooter.openLatch();
        ShootingConstants.ShooterOutputs shooterOutputs = sotm.calculateShooterOutputs(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt);
        shooter.wantedVelocity = shooterSpeed;
        shooter.wantedAcceleration = 0;
        shooter.wantedPitch = Math.toRadians(shooterPitchDegrees);
        turret.wantedAngle = shooterOutputs.turretAngle + Math.toRadians(offsetDegrees);
        turret.wantedAngularVelocity = 0;

        double dx = goalPose.getX()-follower.getPose().getX();
        double dy = goalPose.getY()-follower.getPose().getY();

        double dist = Math.hypot(dx, dy);

        follower.update();
        turret.update();
        shooter.update();
        intake.update();

        telemetry.addData("distance", dist);
        telemetry.addData("shooter speed", shooter.currentVelocity);
        telemetry.addData("pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void init() {
        sotm = new SOTMUtil(goalPose);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        turret = new Turret(hardwareMap);
        shooter = new TorqueShooter(hardwareMap);
        intake = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        turret.reset();
        intake.reset();
        shooter.reset();

        intake.start();
        turret.start();
        shooter.start();

        shooter.openLatch();
    }
}
