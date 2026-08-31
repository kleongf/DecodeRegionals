package org.firstinspires.ftc.teamcode.util.controllers;

public class PIDSVGController {
    private double kP, kI, kD, kS, kV, kG;
    private double current;
    private double target;
    private double minIntegral, maxIntegral;
    private double totalError;
    private double prevError;
    private double lastTimeStamp;
    private double period;

    public PIDSVGController(double kP, double kI, double kD, double kS, double kV, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        reset();
    }

    public void reset() {
        totalError = 0;
        prevError = 0;
        lastTimeStamp = 0;
        period = 0;
    }

    // dont need a feedforward type
    // ex: set kg and multiply kg by cosine. show this in the docs

    public double calculate(double current) {
        return calculate(current, target);
    }

    public double calculate(double current, double target) {
        this.current = current;
        this.target = target;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        totalError += period * (target - current);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        double error = target - current;
        double iError = totalError;
        double dError = period > 1E-6 ? ((error - prevError) / period) : 0;

        prevError = error;

        return kP * error + kI * iError + kD * dError + kS * Math.signum(error) + kV * target + kG;
    }

    // getters and setters

    public double getCurrent() {
        return current;
    }

    public double getTarget() {
        return target;
    }

    public void setCurrent(double current) {
        this.current = current;
    }

    public void setTarget(double target) {
        this.target = target;
    }
    public void setPIDSVGCoefficients(double kP, double kI, double kD, double kS, double kV, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;
    }

    public void setP(double kP) {this.kP = kP;}
    public void setI(double kI) {
        this.kI = kI;
    }
    public void setD(double kD) {
        this.kD = kD;
    }
    public void setS(double kS) {this.kS = kS;}
    public void setV(double kV) {this.kV = kV;}
    public void setG(double kG) {this.kG = kG;}

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }
    public double getS() {return kS;}
    public double getV() {return kV;}
    public double getG() {return kG;}

    public double getPeriod() {return period;}

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    // goofy functions that might be helpful
    public boolean atTarget(double tolerance) {
        return Math.abs(target - current) <= tolerance;
    }
    public void clearTotalError() {
        totalError = 0;
    }
}
