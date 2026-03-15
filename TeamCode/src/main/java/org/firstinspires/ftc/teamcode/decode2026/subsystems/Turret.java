package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.decode2026.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;

public class Turret extends Subsystem {
    public enum Mode {
        TURRET_ON,
        TURRET_OFF
    }
    public double currentPositionTicks;
    public double currentVelocityTicks;
    public double wantedAngle;
    public double offset;
    public double angularVelocityToGoal;
    public Mode wantedMode;
    private final DcMotorEx turretMotor;
    private final AnalogInput externalEncoder;
    private final PIDFController turretController;
    private double kV = TurretConstants.kV;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        externalEncoder = hardwareMap.get(AnalogInput.class, "externalEncoder");

        turretController = new PIDFController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
    }

    @Override
    public void reset() {
        wantedMode = Mode.TURRET_OFF;
        resetEncoderWithAbsoluteReading();
    }

    @Override
    public void start() {
        wantedMode = Mode.TURRET_ON;
    }

    @Override
    public void update() {
        if (TurretConstants.useExternalEncoder) {
            currentPositionTicks = calculatePositionTicks(externalEncoder.getVoltage());
        } else {
            currentPositionTicks = turretMotor.getCurrentPosition() + offset;
        }
        currentVelocityTicks = turretMotor.getVelocity();

        switch (wantedMode) {
            case TURRET_ON:
                double t = weirdAngleWrap(wantedAngle) * TurretConstants.ticksPerRadian;
                double power = turretController.calculate(currentPositionTicks, t);
                double error = t-currentPositionTicks;
                if (Math.abs(error) > TurretConstants.epsilonTicks) {
                    power += TurretConstants.kS * Math.signum(error); // kS so that it works better, lots of friction but this is
                }

                power += kV * angularVelocityToGoal;

                if (Math.abs(power) > TurretConstants.maxPower) {
                    power = TurretConstants.maxPower * Math.signum(power);
                }

                turretMotor.setPower(power);
                break;
            case TURRET_OFF:
                break;
        }
    }
    public void resetMotorEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoderWithAbsoluteReading() {
        offset = calculatePositionTicks(externalEncoder.getVoltage()) - turretMotor.getCurrentPosition();
    }

    public boolean atTarget(double threshold) {
        return Math.abs(turretMotor.getCurrentPosition()-weirdAngleWrap(wantedAngle) * TurretConstants.ticksPerRadian) < threshold;
    }

    public double weirdAngleWrap(double radians) {
        while (radians > 0) {
            radians -= 2 * Math.PI;
        }
        while (radians < -2 * Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
    private double calculatePositionTicks(double voltage) {
        double realOffset = Math.toRadians(TurretConstants.encoderOffsetDegrees + 360);
        double position = ((Math.max(0, voltage-TurretConstants.encoderMinVoltage) / TurretConstants.encoderMaxVoltage) * 2 * Math.PI) % (2 * Math.PI) + realOffset;
        double posToTicksGeared = position * TurretConstants.ticksPerRadian * TurretConstants.encoderGearRatio;

        return posToTicksGeared - 2 * TurretConstants.ticksPerRevolution;
    }

    public void setPDCoefficients(double p, double d) {
        turretController.setPIDF(p, 0, d, 0);
    }

    public void setkV(double kv) {
        kV = kv;
    }
}
