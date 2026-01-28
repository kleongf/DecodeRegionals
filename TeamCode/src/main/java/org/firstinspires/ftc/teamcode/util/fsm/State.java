package org.firstinspires.ftc.teamcode.util.fsm;

import com.pedropathing.util.Timer;

import java.util.ArrayList;
import java.util.List;

public class State {
    private final List<Transition> transitions = new ArrayList<>();
    private final List<TimedRunnable> timedRunnables = new ArrayList<>();
    private int state = -1; // -1 = done, 0 = running
    private final String name;
    private Runnable entryRunnable;
    private Runnable exitRunnable;
    private double minTime = 0;
    private double maxTime = Double.POSITIVE_INFINITY;
    private String nextState; // fallback or transition result
    private final Timer timer = new Timer();

    public State() {
        this.name = null;
    }

    public State(String name) {
        this.name = name;
    }

    public void start() {
        timer.resetTimer();
        state = 0;
        for (TimedRunnable runnable: timedRunnables) {
            runnable.setRun(false);
        }
        for (Transition transition : transitions) {
            transition.start();
        }
        if (entryRunnable != null) {
            entryRunnable.run();
        }
    }

    public void run() {
        if (state != 0) return;

        for (Transition transition : transitions) {
            transition.run(); // <-- Required to evaluate condition
            if (transition.isFinished() && timer.getElapsedTime() >= minTime) {
                nextState = transition.getNextState();
                if (exitRunnable != null) exitRunnable.run();
                state = -1;
                return;
            }
        }
        // have a list of timed runnables.
        for (TimedRunnable runnable: timedRunnables) {
            if (runnable.getMs() > timer.getElapsedTime() && !runnable.wasRun()) {
                runnable.getRunnable().run();
                runnable.setRun(true);
            }
        }

        if (timer.getElapsedTime() >= maxTime) {
            if (exitRunnable != null) exitRunnable.run();
            state = -1;
            // nextState should already be fallback if one was set
        }
    }

    public State transition(Transition transition) {
        transitions.add(transition);
        return this;
    }

    public State onEnter(Runnable runnable) {
        this.entryRunnable = runnable;
        return this;
    }

    public State onExit(Runnable runnable) {
        this.exitRunnable = runnable;
        return this;
    }

    public State minTime(double minTime) {
        this.minTime = minTime;
        return this;
    }

    public State maxTime(double maxTime) {
        this.maxTime = maxTime;
        return this;
    }

    public State fallbackState(String fallbackState) {
        if (fallbackState == null) throw new IllegalArgumentException("Fallback state cannot be null");
        this.nextState = fallbackState;
        return this;
    }

    public State addTimedRunnable(TimedRunnable r) {
        timedRunnables.add(r);
        return this;
    }

    public String getNextState() {
        return nextState;
    }

    public String getName() {
        return name;
    }

    public boolean isFinished() {
        return state == -1;
    }
}

