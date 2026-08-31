package org.firstinspires.ftc.teamcode.decode2026.commands;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.lib.robot.Command;
import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;

public class PrepareShootCommandLonger extends Command {
    private final CurrentRobot robot;
    public PrepareShootCommandLonger(CurrentRobot robot) {
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
                        .maxTime(350),//longer than 200ms, ts is actually the only difference, kind of, see line 28
                new State()
                        .onEnter(() -> {
                            robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                        })
                        .maxTime(200)//and this is longer so don't shoot ball accidentally?
                        //.onExit(robot.shooter::openLatch)

        );
    }
}
