package org.firstinspires.ftc.teamcode.lib.robot;

public abstract class Subsystem {
    public Subsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }
    public void init() {}
    public void start() {}
    public void periodic() {}
}
