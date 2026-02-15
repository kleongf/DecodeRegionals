package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.teleop.MainTeleop;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;

@Config
@TeleOp(name="SOTM teleop tuning turret pid: make it follow goal BETTER! TUNE THIS TOMORROW", group="!")
public class SOTMTeleop3 extends OpMode {
    // steps in order:
    // tune the pid first, so that we can track a moving target.
    // to do this set the latency scale factor to -1
    // when we stop moving, the turret should also stop. it should track the goal well.
    // when we have a good pid, then we know that offset is next
    // tune offset for various distances on the field while recording the angle to the goal
    // find some function (if it is a function) to calculate offset
    // lastly tune the latency scale factor so that the turret and shooter update
    private MainTeleop teleop;
    private Pose startPose = PoseConstants.BLUE_STANDARD_START_POSE;
    public static double p = 0.005; // tune
    public static double d = 0.000; // tune
    public static double offset = 10; // tune, offset for the turret itself
    public static double latencyScaleFactor = 1; // small number prob between 0 and 1
    public static double kF = 0.001; // TODO: still don't know if its going the right direction lol
    // could set to something lower given that we have a p controller maybe like 0.05

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new MainTeleop(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, gamepad2, true);
    }

    @Override
    public void loop() {
        teleop.robot.turret.setPDCoefficients(p, d);
        teleop.sotm.latencyScaleFactor = latencyScaleFactor;
        teleop.sotm.offsetFactor = offset;
        teleop.sotm.kF = kF;
        teleop.loop();
        // graphing these could make it a lot easier to tune.
        telemetry.addData("robot velocity magnitude unweighted", teleop.drivetrain.getVelocity().getMagnitude());
        telemetry.addData("robot velocity magnitude weighted", teleop.getRollingVelocity().getMagnitude());
        telemetry.addData("current pos", teleop.robot.turret.getCurrent());
        telemetry.addData("target pos", teleop.robot.turret.getTarget());
        telemetry.update();
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
