package org.firstinspires.ftc.teamcode.lib.robot;

public abstract class Robot {
    protected final CommandScheduler scheduler;

    public Robot() {
        CommandScheduler.getInstance().reset();
        this.scheduler = CommandScheduler.getInstance();
    }

    public void init() {}
    public void start() {}

    public void periodic() {
        scheduler.run();
    }
}
