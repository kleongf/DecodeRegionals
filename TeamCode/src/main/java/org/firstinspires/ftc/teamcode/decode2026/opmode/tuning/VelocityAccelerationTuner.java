package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;

@Config
@TeleOp(name="velocity accel tuner")
public class VelocityAccelerationTuner extends OpMode {
    private Follower follower;
    private final Pose startPose = FieldConstants.RED_CLOSE_START_AUTO_POSE;
    private double power = 0;
    private boolean on = true;


    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            on = false;
        }

        if (on) {
            follower.setTeleOpDrive(1, 0, 0);
        } else {
            follower.setTeleOpDrive(0, 0, 0);
        }

        follower.update();
        telemetry.addLine(follower.getPose().toString());
        telemetry.update();

        Log.d("Velocity", String.valueOf(follower.getVelocity().getMagnitude()));
        Log.d("Acceleration", String.valueOf(follower.getAcceleration().getMagnitude()));
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive(true);
    }

    @Override
    public void start() {
    }
}
