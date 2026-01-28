package org.firstinspires.ftc.teamcode.util.fsm;

public class TimedRunnable {
    private Runnable runnable;
    private int ms;
    private boolean run;
    public TimedRunnable(int ms, Runnable runnable) {
        this.ms = ms;
        this.runnable = runnable;
        this.run = false;
    }

    public int getMs() {
        return ms;
    }

    public Runnable getRunnable() {
        return runnable;
    }

    public void setRun(boolean wasRun) {
        this.run = wasRun;
    }

    public boolean wasRun() {
        return run;
    }
}
