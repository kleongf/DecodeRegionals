package org.firstinspires.ftc.teamcode.decode2026.commands;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.lib.robot.Command;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

public class PrepareShootCommand extends Command {
    private final CurrentRobot robot;
    public PrepareShootCommand(CurrentRobot robot) {
        this.robot = robot;
    }

    @Override
    public StateMachine build() {
        return new StateMachine(
                new State()
                        .onEnter(() -> {
                            robot.intake.wantedMode = Intake.Mode.INTAKE_FAST;
                            robot.shooter.closeLatch();
                        })
                        .maxTime(200),
                new State()
                        .onEnter(() -> {
                            robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                        })
                        .maxTime(100)
                        .onExit(robot.shooter::openLatch)

        );
    }
}
