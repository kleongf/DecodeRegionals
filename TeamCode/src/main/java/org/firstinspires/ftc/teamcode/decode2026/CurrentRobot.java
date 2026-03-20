package org.firstinspires.ftc.teamcode.decode2026;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode2026.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.decode2026.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.decode2026.commands.ShootCommandSlow;
import org.firstinspires.ftc.teamcode.lib.robot.Robot;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.CameraLocalizer;
import org.firstinspires.ftc.teamcode.lib.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import java.util.ArrayList;

public class CurrentRobot extends Robot {
    private final BulkRead bulkRead;
    private final ArrayList<Subsystem> subsystems;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final LEDIndicator ledIndicator;
    public final CameraLocalizer cameraLocalizer;

    private final ArrayList<StateMachine> commands;
    public StateMachine intakeCommand;
    public StateMachine shootCommand;
    public StateMachine shootCommandSlow;
    private ElapsedTime loopTimer;
    public double dt;

    public CurrentRobot(HardwareMap hardwareMap) {
        loopTimer = new ElapsedTime();

        bulkRead = new BulkRead(hardwareMap);
        subsystems = new ArrayList<>();

        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);

        turret = new Turret(hardwareMap);
        subsystems.add(turret);

        ledIndicator = new LEDIndicator(hardwareMap);
        subsystems.add(ledIndicator);

        cameraLocalizer = new CameraLocalizer(hardwareMap);
        subsystems.add(cameraLocalizer);

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
        dt = loopTimer.seconds() <= 0 ? 0.02 : loopTimer.seconds();
        loopTimer.reset();
        bulkRead.clearCache();

        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
        for (StateMachine command : commands) {
            command.update();
        }
    }
}
