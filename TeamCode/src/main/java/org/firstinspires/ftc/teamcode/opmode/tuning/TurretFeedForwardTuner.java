package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.teleop.MainTeleop;
import org.firstinspires.ftc.teamcode.opmode.teleop.MainTeleop2;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;

@Config
@TeleOp(name="BEST and ONLY sotm tuner: p, d, offset, kf")
public class TurretFeedForwardTuner extends OpMode {
    private MainTeleop2 teleop;
    private Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    public static double p = 0.002; // tune
    public static double d = 0.0001; // tune
    public static double offset = 0;
    public static double kF = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new MainTeleop2(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, gamepad2, true);
    }

    @Override
    public void loop() {
        teleop.sotm.kF = kF;
        teleop.sotm.offsetFactor = offset;
        teleop.robot.turret.setPDCoefficients(p, d);
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
