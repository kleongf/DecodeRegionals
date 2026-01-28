package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

import java.util.ArrayList;

public class AutonomousRobot {
    private final ArrayList<Subsystem> subsystems;
    public final BulkRead bulkRead;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final ArtifactVision vision;

    private final ArrayList<StateMachine> commands;
    public StateMachine intakeCommand;
    public StateMachine shootCommand;
    public StateMachine shootCommandSlow;

    public AutonomousRobot(HardwareMap hardwareMap, Alliance alliance) {
        subsystems = new ArrayList<>();

        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);

        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);

        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        subsystems.add(turret);

        vision = new ArtifactVision(hardwareMap, alliance);
        subsystems.add(vision);

        commands = new ArrayList<>();

        intakeCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(intakeCommand);

        shootCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                            shooter.openLatch();
                        })
                        .maxTime(10),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .maxTime(600));
        commands.add(shootCommand);

        shootCommandSlow = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                            shooter.openLatch();
                        })
                        .maxTime(150),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .maxTime(1500));
        commands.add(shootCommandSlow);

        // meant to be called _ seconds (usually 0.8?) into a path.
    }

    public void initPositions() {
        // configure shooter
        shooter.state = Shooter.ShooterState.SHOOTER_ON;
        shooter.closeLatch();
        shooter.setTargetVelocity(0);
        shooter.setShooterPitch(Math.toRadians(0));
        // configure turret
        turret.setTarget(0);
        // configure intake
        intake.state = Intake.IntakeState.INTAKE_OFF;
    }

    public void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
        for (StateMachine command : commands) {
            command.update();
        }
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
        }
    }

    public void setAzimuthThetaVelocity(double[] values) {
        turret.setTarget(values[0]);
        shooter.setShooterPitch(values[1]);
        shooter.setTargetVelocity(values[2]);
    }
}