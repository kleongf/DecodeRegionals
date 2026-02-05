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
    private MainTeleop teleop;
    private Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    public static double p = 0.005; // tune
    public static double d = 0.0003; // tune
    public static double offset = 0.0; // tune, offset for the turret itself
    public static double latencyScaleFactor = 0; // small number prob between 0 and 1

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
        teleop.loop();
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
