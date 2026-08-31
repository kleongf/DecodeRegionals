package org.firstinspires.ftc.teamcode.lib.robot;

import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;

import java.util.List;

public class Command {
    private final Robot robot;
    private final List<Subsystem> requirements;
    private final StateMachine stateMachine;

    public Command(Robot robot, List<Subsystem> requirements, StateMachine stateMachine) {
        this.robot = robot;
        this.requirements = requirements;
        this.stateMachine = stateMachine;
    }

    public List<Subsystem> getRequirements() {
        return requirements;
    }

    public boolean isFinished() {
        return stateMachine.isFinished();
    }

    void start() {
        stateMachine.start();
    }

    void periodic() {
        stateMachine.periodic();
    }

    void interrupt() {
        stateMachine.interrupt();
    }
}
