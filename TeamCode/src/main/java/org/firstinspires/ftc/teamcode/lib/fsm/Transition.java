package org.firstinspires.ftc.teamcode.util.fsm;

import java.util.function.Supplier;

public class Transition {
    private int state; // -1 = finished, 0 = active
    private final Supplier<Boolean> condition;
    private final String nextState;

    public Transition(Supplier<Boolean> condition, String nextState) {
        this.condition = condition;
        this.nextState = nextState;
        this.state = -1;
    }

    public Transition(Supplier<Boolean> condition) {
        this(condition, null);
    }

    public void start() {
        state = 0;
    }

    public void run() {
        if (state == 0 && condition.get()) {
            state = -1;
        }
    }

    public String getNextState() {
        return nextState;
    }

    public boolean isFinished() {
        return state == -1;
    }
}

