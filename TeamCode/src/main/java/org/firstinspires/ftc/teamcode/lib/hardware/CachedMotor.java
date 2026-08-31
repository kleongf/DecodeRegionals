package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedMotor {
    public DcMotorEx motorEx;

    // The minimum difference between the current and requested motor power between motor writes
    private double cachingTolerance = 0.0001;
    private double lastPower = 0;

    public CachedMotor(DcMotorEx motorEx) {
        this.motorEx = motorEx;
    }

    /**
     * @return the caching tolerance of the motor before it writes a new power to the motor
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /**
     * @param cachingTolerance the new caching tolerance between motor writes
     * @return this object for chaining purposes
     */
    public CachedMotor setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /**
     * @param power power to be assigned to the motor if difference is greater than caching tolerance or if power is exactly 0
     */
    public void setPower(double power) {
        if ((Math.abs(power - lastPower) > cachingTolerance) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motorEx.setPower(power);
        }
    }

    public double getCurrent() {
        return motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public double getVelocity() {
        return motorEx.getVelocity();
    }

    public double getPosition() {
        return motorEx.getCurrentPosition();
    }
}
