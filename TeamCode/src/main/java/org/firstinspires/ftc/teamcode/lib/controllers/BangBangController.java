package org.firstinspires.ftc.teamcode.util.controllers;

public class BangBangController {
    private double maxOutput;
    public BangBangController() {
        maxOutput = 1.0;
    }
    public BangBangController(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double calculate(double current, double target) {
        if (current < target) {
            return maxOutput;
        }
        return 0;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }
}
