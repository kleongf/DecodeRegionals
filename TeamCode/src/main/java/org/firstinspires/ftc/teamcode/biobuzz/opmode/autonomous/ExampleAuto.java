package org.firstinspires.ftc.teamcode.biobuzz.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.biobuzz.CurrentRobot;
import org.firstinspires.ftc.teamcode.biobuzz.commands.TurnToHeadingCommand;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.lib.fsm.Transition;
import org.firstinspires.ftc.teamcode.lib.robot.Command;
import org.firstinspires.ftc.teamcode.lib.robot.CommandScheduler;

@Autonomous(name = "Example Auto", group = "!")
public class ExampleAuto extends OpMode {
    private CurrentRobot robot;
    private StateMachine stateMachine;
    private Command turnCommand;

    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose scorePose = new Pose(24, 24, Math.toRadians(90));
    private final Pose parkPose = new Pose(12, 48, 0);

    private PathChain driveToScore;
    private PathChain driveToPark;

    private void buildPaths() {
        Follower follower = robot.drivetrain.follower;

        driveToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        // this auto only drives via followPath/TurnToHeadingCommand, never manual joystick
        // input, so the alliance-dependent heading offset (see Drivetrain) has no effect here
        robot = new CurrentRobot(hardwareMap, Alliance.BLUE);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.init();
        buildPaths();

        stateMachine = new StateMachine(
                new State("driveToScore")
                        .onEnter(() -> robot.drivetrain.follower.followPath(driveToScore, true))
                        .transition(new Transition(() -> !robot.drivetrain.follower.isBusy())),
                new State("holdHeading")
                        .onEnter(() -> {
                            turnCommand = new TurnToHeadingCommand(robot, robot.drivetrain, scorePose.getHeading());
                            CommandScheduler.getInstance().schedule(turnCommand);
                        })
                        .transition(new Transition(() -> turnCommand.isFinished())),
                new State("driveToPark")
                        .onEnter(() -> robot.drivetrain.follower.followPath(driveToPark, true))
                        .transition(new Transition(() -> !robot.drivetrain.follower.isBusy()))
        );
    }

    @Override
    public void start() {
        stateMachine.start();
        robot.start();
    }

    @Override
    public void loop() {
        stateMachine.periodic();
        robot.periodic();

        telemetry.addData("Pose", robot.drivetrain.follower.getPose());
        telemetry.update();
    }
}
