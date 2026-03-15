package org.firstinspires.ftc.teamcode.lib.robot;

import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

public abstract class Command {
    public Command() {}
    public StateMachine build() {
        return new StateMachine();
    }
}
