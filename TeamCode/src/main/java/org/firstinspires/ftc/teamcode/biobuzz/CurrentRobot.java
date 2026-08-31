package org.firstinspires.ftc.teamcode.decode2026;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode2026.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.decode2026.commands.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.decode2026.commands.PrepareShootCommandLonger;
import org.firstinspires.ftc.teamcode.decode2026.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.decode2026.commands.ShootCommandFast;
import org.firstinspires.ftc.teamcode.decode2026.commands.ShootCommandSlow;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.TorqueShooter;
import org.firstinspires.ftc.teamcode.lib.robot.Robot;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.CameraLocalizer;
import org.firstinspires.ftc.teamcode.lib.hardware.BulkRead;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import java.util.ArrayList;

public class CurrentRobot extends Robot {
    private final BulkRead bulkRead;
    private final ArrayList<Subsystem> subsystems;
    public final Intake intake;
    public final TorqueShooter shooter;
    public final Turret turret;
    public final Tilt tilt;
    public final LEDIndicator ledIndicator;
    public final CameraLocalizer cameraLocalizer;
    public final ArtifactVision artifactVision;
    private final ArrayList<StateMachine> commands;
    public StateMachine shootCommandFast;
    public StateMachine intakeCommand;
    public StateMachine shootCommand;
    public StateMachine shootCommandSlow;
    public StateMachine prepareShootCommand;
    public StateMachine prepareShootCommandLonger;
    private final ElapsedTime loopTimer;
    public double dt;

    public CurrentRobot(HardwareMap hardwareMap) {
        loopTimer = new ElapsedTime();

        bulkRead = new BulkRead(hardwareMap);
        subsystems = new ArrayList<>();

        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        shooter = new TorqueShooter(hardwareMap);
        subsystems.add(shooter);

        turret = new Turret(hardwareMap);
        subsystems.add(turret);

        tilt = new Tilt(hardwareMap);
        subsystems.add(tilt);

        ledIndicator = new LEDIndicator(hardwareMap);
        subsystems.add(ledIndicator);

        cameraLocalizer = new CameraLocalizer(hardwareMap);
        subsystems.add(cameraLocalizer);

        artifactVision = new ArtifactVision(hardwareMap);
        subsystems.add(artifactVision);

        commands = new ArrayList<>();
        // this is called last to ensure everything is initialized
        registerCommands();
    }

    private void registerCommands() {
        intakeCommand = new IntakeCommand(this).build();
        commands.add(intakeCommand);

        shootCommand = new ShootCommand(this).build();
        commands.add(shootCommand);

        shootCommandSlow = new ShootCommandSlow(this).build();
        commands.add(shootCommandSlow);

        shootCommandFast = new ShootCommandFast(this).build();
        commands.add(shootCommandFast);

        prepareShootCommand = new PrepareShootCommand(this).build();
        commands.add(prepareShootCommand);

        prepareShootCommandLonger = new PrepareShootCommandLonger(this).build();
        commands.add(prepareShootCommandLonger);
    }

    @Override
    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
        }
        loopTimer.reset();
    }

    @Override
    public void reset() {
        for (Subsystem subsystem : subsystems) {
            subsystem.reset();
        }
        loopTimer.reset();
    }

    @Override
    public void update() {
        dt = loopTimer.seconds() <= 0 ? RobotConstants.dt : loopTimer.seconds();
        loopTimer.reset();
        bulkRead.clearCache();
        // adding this here, might be a bad place to put it though
        turret.flywheelVelocityTicks = shooter.currentVelocity;

        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
        for (StateMachine command : commands) {
            command.update();
        }
    }
}
