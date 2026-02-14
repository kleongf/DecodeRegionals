package org.firstinspires.ftc.teamcode.robot.robots;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

import java.util.ArrayList;

public class TeleopRobot {
    // new idea: how about we stay in the idle state when the thing is !isFinished? or something. need better method
    // or be able to call the stop multipossess on command, which is the main problem
    // intake slow is 0.8 while intake fast is 1 i dont think it really makes a difference
    private final ArrayList<Subsystem> subsystems;
    public final BulkRead bulkRead;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final LimelightLocalizer limelightLocalizer;
    public final Pivot pivot;

    private final ArrayList<StateMachine> commands;
    public StateMachine shootCommand;
    public StateMachine idleCommand;

    public TeleopRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();

        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);

        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);

        turret = new Turret(hardwareMap);
        subsystems.add(turret);

        limelightLocalizer = new LimelightLocalizer(hardwareMap);
        limelightLocalizer.setPipeline(LimelightLocalizer.Pipeline.APRILTAG);
        subsystems.add(limelightLocalizer);

        pivot = new Pivot(hardwareMap);
        subsystems.add(pivot);

        commands = new ArrayList<>();

        shootCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                            shooter.openLatch();
                        })
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .onExit(() -> {
                            shooter.closeLatch();
                            intake.resetDetection();
                        })
                        .maxTime(800)
//                new State()
//                        .onEnter(() -> {
//                            intake.state = Intake.IntakeState.INTAKE_FAST;
//                            shooter.closeLatch();
//                            intake.resetDetection();
//                        })
//                        .maxTime(100)
        );
        commands.add(shootCommand);

        idleCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_SLOW;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(idleCommand);
    }

    public void setAzimuthThetaVelocity(double[] values) {
        turret.setTarget(values[0]);
        shooter.setShooterPitch(values[1]);
        shooter.setTargetVelocity(values[2]);
    }

    public void initPositions() {
        // configure shooter
        shooter.state = Shooter.ShooterState.SHOOTER_ON;
        shooter.closeLatch();
        limelightLocalizer.setPipeline(LimelightLocalizer.Pipeline.APRILTAG);
    }

    public void update() {
        for (Subsystem subsystem: subsystems) {
            subsystem.update();
        }
        for (StateMachine command: commands) {
            command.update();
        }
    }

    public void start() {
        for (Subsystem subsystem: subsystems) {
            subsystem.start();
        }
    }
}

