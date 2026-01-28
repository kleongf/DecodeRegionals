package org.firstinspires.ftc.teamcode.util.fsm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

public class StateMachine {
    private final List<State> states;
    private int index;
    private boolean running;

    public StateMachine(State... states) {
        this.states = new ArrayList<>(Arrays.asList(states));
        this.index = 0;
        this.running = false;
    }

    public void start() {
        if (!states.isEmpty()) {
            index = 0;
            running = true;
            states.get(0).start();
        }
    }

    private int getIndex(String stateName) {
        return IntStream.range(0, states.size())
                .filter(i -> states.get(i).getName().equals(stateName))
                .findFirst()
                .orElse(-1);
    }

    public void update() {
        if (!running) return;

        State current = states.get(index);
        current.run();

        if (current.isFinished()) {
            String nextStateName = current.getNextState();
            int nextIndex = (nextStateName != null) ? getIndex(nextStateName) : index + 1;

            if (nextIndex >= 0 && nextIndex < states.size()) {
                index = nextIndex;
                states.get(index).start();
            } else {
                running = false;
            }
        }
    }

    public boolean isFinished() {
        return !running;
    }
}

