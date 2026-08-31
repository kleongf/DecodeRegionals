package org.firstinspires.ftc.teamcode.biobuzz.opmode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.CurrentRobot;
import org.firstinspires.ftc.teamcode.lib.Alliance;

public class MainTeleop {
    public final CurrentRobot robot;
    private final Telemetry telemetry;
    private final Gamepad gamepad1, gamepad2;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        robot = new CurrentRobot(hardwareMap, alliance);
        robot.drivetrain.follower.setStartingPose(startPose);
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        robot.init();
    }

    public void init_loop() {
    }

    public void start() {
        robot.start();
    }

    public void periodic() {
        robot.drivetrain.driveManual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        robot.periodic();

        telemetry.addData("Pose", robot.drivetrain.follower.getPose());
        telemetry.update();
    }
}
