package org.firstinspires.ftc.teamcode.decode2026.commands;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.lib.robot.Command;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

public class ShootCommand extends Command {
    private final CurrentRobot robot;
    public ShootCommand(CurrentRobot robot) {
        this.robot = robot;
    }

    @Override
    public StateMachine build() {
        return new StateMachine(
                new State()
                        .onEnter(() -> {
                            robot.shooter.openLatch();
                            robot.intake.wantedMode = Intake.Mode.INTAKE_FAST;
                        })
                        .onExit(() -> robot.intake.detectionState = Intake.DetectionState.EMPTY)
                        .maxTime(400)
        );
    }
}
