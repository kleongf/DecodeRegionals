package org.firstinspires.ftc.teamcode.lib.controllers;

public class WeightedSetpointPIDController {

    private double kP, kI, kD;

    // Setpoint weights:
    // beta:  weight on the proportional term. beta < 1 reduces proportional kick on setpoint steps.
    // gamma: weight on the derivative term. gamma < 1 reduces derivative kick on setpoint steps.
    // The integral term carries no setpoint weight (equivalent to c = 1 always).
    private double beta, gamma;

    private double setPoint;
    private double measuredValue;

    private double minIntegral, maxIntegral;

    private double errorVal_p;
    private double errorVal_v;

    private double totalError;
    private double prevMeasuredValue;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    /**
     * Constructs a weighted setpoint PID controller with default weights (beta=1, gamma=1),
     * which is equivalent to a standard PID.
     *
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    public WeightedSetpointPIDController(double kp, double ki, double kd) {
        this(kp, ki, kd, 1.0, 1.0, 0, 0);
    }

    /**
     * Constructs a weighted setpoint PID controller.
     *
     * <p>The control law is:
     * <pre>
     *   u(t) = kP*(beta*sp - pv) + kI*∫(sp - pv)dt + kD*d/dt(gamma*sp - pv)
     * </pre>
     *
     * <p>Setting {@code beta < 1} reduces the proportional response to setpoint steps.
     * Setting {@code gamma < 1} softens the derivative response to setpoint changes,
     * eliminating "derivative kick". Both weights equal to 1.0 reduces this to a standard PID.
     *
     * @param kp    Proportional gain.
     * @param ki    Integral gain.
     * @param kd    Derivative gain.
     * @param beta  Setpoint weight on the proportional term [0, 1].
     * @param gamma Setpoint weight on the derivative term [0, 1].
     * @param sp    Initial setpoint.
     * @param pv    Initial measured process variable.
     */
    public WeightedSetpointPIDController(double kp, double ki, double kd,
                                         double beta, double gamma, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;

        this.beta = beta;
        this.gamma = gamma;

        setPoint = sp;
        measuredValue = pv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = beta * setPoint - measuredValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevMeasuredValue = measuredValue;
        lastTimeStamp = 0;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }

    /**
     * Returns the current setpoint.
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Updates the setpoint without resetting the controller state,
     * allowing bumpless transfer when the target changes.
     *
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        setPoint = sp;
        errorVal_p = beta * setPoint - measuredValue;
    }

    /**
     * Sets the setpoint weights.
     *
     * @param beta  Proportional setpoint weight [0, 1]. Use beta=0 to make the proportional
     *              term act only on -pv, fully eliminating proportional kick.
     * @param gamma Derivative setpoint weight [0, 1]. Use gamma=0 to make the derivative
     *              term act only on -pv, fully eliminating derivative kick.
     */
    public void setWeights(double beta, double gamma) {
        this.beta = beta;
        this.gamma = gamma;
    }

    /**
     * Returns true if the unweighted error (sp - pv) is within the bounds set by
     * {@link #setTolerance}. Uses the unweighted error so that atSetPoint() reflects
     * the true process error regardless of beta.
     */
    public boolean atSetPoint() {
        return Math.abs(setPoint - measuredValue) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
    }

    /**
     * @return the weighted proportional error e_P(t) = beta*sp - pv
     */
    public double getPositionError() {
        return errorVal_p;
    }

    /**
     * @return the velocity error e_D'(t), computed on the weighted derivative signal
     */
    public double getVelocityError() {
        return errorVal_v;
    }

    /**
     * Calculates the next output using the last measured value.
     */
    public double calculate() {
        return calculate(measuredValue);
    }

    /**
     * Calculates the next output given a new setpoint and measured value.
     *
     * @param pv The current process variable measurement.
     * @param sp The desired setpoint.
     */
    public double calculate(double pv, double sp) {
        setSetPoint(sp);
        return calculate(pv);
    }

    /**
     * Calculates the control output u(t) using weighted setpoint PID.
     *
     * <p>Proportional term acts on (beta*sp - pv), so a setpoint step contributes
     * only beta of its magnitude, reducing kick when beta < 1.
     *
     * <p>Derivative term acts on (gamma*sp - pv). Since sp is constant between
     * setSetPoint calls, the derivative reduces to -pv', with gamma scaling any
     * contribution from a setpoint change.
     *
     * <p>Integral term accumulates the unweighted error (sp - pv).
     *
     * @param pv The current measurement of the process variable.
     * @return the control output u(t).
     */
    public double calculate(double pv) {
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        measuredValue = pv;

        // Proportional on (beta*sp - pv)
        errorVal_p = beta * setPoint - measuredValue;

        // Derivative on (gamma*sp - pv)
        double weightedDerivativeSignal = gamma * setPoint - measuredValue;
        double prevWeightedDerivativeSignal = gamma * setPoint - prevMeasuredValue;

        if (Math.abs(period) > 1E-6) {
            errorVal_v = (weightedDerivativeSignal - prevWeightedDerivativeSignal) / period;
        } else {
            errorVal_v = 0;
        }
        prevMeasuredValue = measuredValue;

        // Integral of unweighted error (sp - pv)
        totalError += period * (setPoint - measuredValue);
        totalError = Math.max(minIntegral, Math.min(maxIntegral, totalError));

        return kP * errorVal_p + kI * totalError + kD * errorVal_v;
    }

    public void setPID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    public void clearTotalError() {
        totalError = 0;
    }

    public double getPeriod() {
        return period;
    }
}
