package org.firstinspires.ftc.teamcode.opmode.comp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

@TeleOp(name="Robot Subsystem Test", group="comp")
public class RobotTest extends OpMode {
    private AutonomousRobot robot;
    private Gamepad gp1;
    private StateMachine testTurret;
    private StateMachine testShooter;
    private StateMachine testLatch;
    private StateMachine testIntake;

    @Override
    public void init() {
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        gp1 = gamepad1;

        testTurret = new StateMachine(
             new State()
                     .onEnter(() -> robot.turret.setTarget(Math.toRadians(45)))
                     .maxTime(3000),
                new State()
                        .onEnter(() -> robot.turret.setTarget(0))
                        .maxTime(3000)
        );

        testShooter = new StateMachine(
                new State()
                        .onEnter(() -> robot.shooter.shooterMotor.setPower(1))
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.shooter.shooterMotor.setPower(0))
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.shooter.shooterMotor2.setPower(1))
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.shooter.shooterMotor2.setPower(0))
                        .maxTime(2000)

//                new State()
//                        .onEnter(() -> robot.shooter.setTargetVelocity(2500))
//                        .maxTime(3000),
//                new State()
//                        .onEnter(() -> robot.shooter.setTargetVelocity(0))
//                        .maxTime(3000)
        );

        testLatch = new StateMachine(
                new State()
                        .onEnter(() -> robot.shooter.openLatch())
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.shooter.closeLatch())
                        .maxTime(2000)
        );


        testIntake = new StateMachine(
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(3000),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_FAST)
                        .maxTime(3000),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .maxTime(3000)
        );
    }

    @Override
    public void loop() {

        if (gp1.aWasPressed()) {
            testTurret.start();
        }

        if (gp1.bWasPressed()) {
            testShooter.start();
        }

        if (gp1.xWasPressed()) {
            testLatch.start();
        }

        if (gp1.yWasPressed()) {
            testIntake.start();
        }

        testTurret.update();
        testShooter.update();
        testLatch.update();
        testIntake.update();

        telemetry.addData("shooter velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("intake velocity", robot.intake.intakeMotor.getVelocity());

        // robot.update();
        // robot.shooter.update();
        robot.intake.update();
        robot.turret.update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.initPositions();
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;
        robot.start();
    }
}

