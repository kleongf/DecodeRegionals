package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.subsystems.ArtifactVision2;
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
    public final ArtifactVision2 vision;
    private final Servo light;

    private final ArrayList<StateMachine> commands;
    public StateMachine intakeCommand;
    public StateMachine shootCommand;
    public StateMachine shootCommandSlow;

    public AutonomousRobot(HardwareMap hardwareMap, Alliance alliance) {
        subsystems = new ArrayList<>();
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(0);

        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);

        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);

        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        subsystems.add(turret);

        vision = new ArtifactVision2(hardwareMap, alliance);
        subsystems.add(vision);

        commands = new ArrayList<>();

        // putting it here because why not. prob should go after shooting but whatever, this gives us more time
        // besides intake is always called right after shoot
        intakeCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        .maxTime(200),
                new State()
                        .onEnter(() -> {
                            intake.resetDetection();
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(intakeCommand);

        shootCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            shooter.openLatch();
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        .maxTime(400)); // new time apparently
        commands.add(shootCommand);

        shootCommandSlow = new StateMachine(
                new State()
                        .onEnter(() -> {
                            shooter.openLatch();
                            intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .maxTime(600)); // new time apparently
        commands.add(shootCommandSlow);

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