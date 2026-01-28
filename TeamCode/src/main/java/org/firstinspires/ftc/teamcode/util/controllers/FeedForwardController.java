package org.firstinspires.ftc.teamcode.util.controllers;

// Adapted from https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
// according to it, best values are achieved with a proportional controller + feedforward controller
// this makes it quick to disturbances like shooting a ball


public class FeedForwardController {
    private double kp;
    private double ks;
    private double kv;
    public FeedForwardController(double kv, double ks, double kp) {
        this.kp = kp;
        this.ks = ks;
        this.kv = kv;
    }

    public double calculate(double current, double target) {
        // we will probably not use ks
        return kv * target + kp * (target-current) + ks * Math.signum(target-current);
    }

    public void setCoefficients(double kv, double ks, double kp) {
        this.kp = kp;
        this.ks = ks;
        this.kv = kv;
    }
}
