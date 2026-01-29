package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;

@TeleOp(name="Blue Teleop (reset encoders)", group="!")
public class BlueTeleop extends OpMode {
    private MainTeleop teleop;
    private Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    @Override
    public void init() {
        teleop = new MainTeleop(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, gamepad2,true);
    }

    @Override
    public void loop() {
        teleop.loop();
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
