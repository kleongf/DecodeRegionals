package org.firstinspires.ftc.teamcode.util.controllers;

import com.pedropathing.util.Timer;

public class PDFLController {
    private double kP, kD, kF, kL;
    private double maxPower;
    private double tolerance;
    private double prevErrorVal;
    private double target;
    private double period;

    private final Timer timer;

    public PDFLController(double kp, double kd, double kf, double kl, double mp, double t) {
        kP = kp;
        kD = kd;
        kF = kf;
        kL = kl;
        tolerance = t;
        maxPower = mp;
        timer = new Timer();
        reset();
    }

    public void reset() {
        prevErrorVal = 0;
        timer.resetTimer();
    }

    public double calculate(double current) {
        // Ensure non-zero dt
        period = Math.max(timer.getElapsedTimeSeconds(), 1e-5);

        double error = target - current;
        double dError = (error - prevErrorVal) / period;

        prevErrorVal = error;
        timer.resetTimer();

        // only apply kL if there's an error
        double lComponent = (Math.abs(error) >= tolerance) ? kL : 0;
        double total = kP * error + kD * dError + kF + lComponent;

        // Clamp to max power
        if (Math.abs(total) > maxPower) {
            return Math.signum(total) * maxPower;
        }

        return total;
    }

    public void setTarget(double t) {
        target = t;
    }

    public double getTarget() {
        return target;
    }

    public double getError(double current) {
        return target - current;
    }

    public boolean atTarget(double current) {
        return Math.abs(target - current) <= tolerance;
    }

    public double[] getCoefficients() {
        return new double[]{kP, kD, kF, kL};
    }

    public void setCoefficients(double kp, double kd, double kf, double kl, double mp, double t) {
        kP = kp;
        kD = kd;
        kF = kf;
        kL = kl;
        maxPower = mp;
        tolerance = t;
    }

    public void setP(double kp) { kP = kp; }
    public void setD(double kd) { kD = kd; }
    public void setF(double kf) { kF = kf; }
    public void setL(double kl) { kL = kl; }

    public double getP() { return kP; }
    public double getD() { return kD; }
    public double getF() { return kF; }
    public double getL() { return kL; }

    public void setMaxPower(double mp) { maxPower = mp; }
    public double getMaxPower() { return maxPower; }
    public double getTolerance() { return tolerance; }
    public void setTolerance(double t) { tolerance = t; }
    public double getPeriod() { return period; }
}

