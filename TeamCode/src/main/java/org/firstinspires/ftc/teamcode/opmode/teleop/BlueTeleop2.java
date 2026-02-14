package org.firstinspires.ftc.teamcode.opmode.teleop;

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
@TeleOp(name="Blue Teleop Scrim", group="!")
public class BlueTeleop2 extends OpMode {
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

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new MainTeleop(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, gamepad2, true);
    }

    @Override
    public void loop() {
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
