package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode2026.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.controllers.WeightedSetpointPIDController;
import org.firstinspires.ftc.teamcode.util.decodeutil.CachedMotor;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

public class Turret extends Subsystem {
    public enum Mode {
        TURRET_ON,
        TURRET_OFF
    }
    public double currentPositionTicks;
    public double wantedPositionTicks;
    public double currentVelocityTicks;
    public double wantedAngle;
    public double currentAngle;
    public double offset;
    public double errorTicks;
    public double wantedAngularVelocity;
    public Mode wantedMode;
    public double flywheelVelocityTicks;
    private final DcMotorEx turretMotor;
    private final AnalogInput externalEncoder;
    private final VoltageSensor voltageSensor;
    private final PIDFController turretController;
    private final WeightedSetpointPIDController turretControllerWeighted;
    private double prevSetPower = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        externalEncoder = hardwareMap.get(AnalogInput.class, "externalEncoder");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        turretController = new PIDFController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        turretControllerWeighted = new WeightedSetpointPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    }

    @Override
    public void reset() {
        wantedMode = Mode.TURRET_OFF;
        resetEncoderWithAbsoluteReading();
        // resetMotorEncoder();
    }

    @Override
    public void start() {
        wantedMode = Mode.TURRET_ON;
    }

    @Override
    public void update() {
        turretController.setIntegrationBounds(-0.2, 0.2);
        turretController.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        turretControllerWeighted.setPID(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        turretControllerWeighted.setWeights(TurretConstants.beta, TurretConstants.gamma);

        if (TurretConstants.useExternalEncoder) {
            currentPositionTicks = calculatePositionTicks(externalEncoder.getVoltage());
        } else {
            currentPositionTicks = turretMotor.getCurrentPosition() + offset;
        }

        currentAngle = turretMotor.getCurrentPosition() / TurretConstants.ticksPerRadian;
        currentVelocityTicks = turretMotor.getVelocity();
        wantedPositionTicks = weirdAngleWrap(wantedAngle) * TurretConstants.ticksPerRadian;

        switch (wantedMode) {
            case TURRET_ON:
                if (TurretConstants.useLQRController) {
                    double t = weirdAngleWrap(wantedAngle) * TurretConstants.ticksPerRadian;
                    errorTicks = t-currentPositionTicks;

                    double LQRErrorPosition = currentPositionTicks - t;
                    double LQRErrorVelocity = currentVelocityTicks - wantedAngularVelocity * TurretConstants.ticksPerRadian;

                    // a lqr controller follows the model u = -K(error)
                    double u = -(TurretConstants.kPos * LQRErrorPosition + TurretConstants.kVel * LQRErrorVelocity);
                    u += TurretConstants.kS * Math.signum(u);

                    double power = MathUtil.clamp(
                            u,
                            -TurretConstants.maxPower,
                            TurretConstants.maxPower
                    );
                    if (TurretConstants.useVoltageCompensation) {
                        power *= (TurretConstants.nominalVoltage / voltageSensor.getVoltage());
                    }

//                    if (Math.abs(prevSetPower - power) > 0.03) {
//                        turretMotor.setPower(power);
//                        prevSetPower = power;
//                    }

                    turretMotor.setPower(power);
                } else if (TurretConstants.useWeightSetpointPID) {
                    double t = weirdAngleWrap(wantedAngle) * TurretConstants.ticksPerRadian;
                    double error = t-currentPositionTicks;
                    errorTicks = error;

                    double power = MathUtil.clamp(
                            turretControllerWeighted.calculate(currentPositionTicks, t) +
                                    TurretConstants.kS * Math.signum(error) +
                                    TurretConstants.kV * wantedAngularVelocity
                            ,
                            -TurretConstants.maxPower,
                            TurretConstants.maxPower
                    );
                    if (TurretConstants.useVoltageCompensation) {
                        power *= (TurretConstants.nominalVoltage / voltageSensor.getVoltage());
                    }

                    turretMotor.setPower(power);
                } else {
                    double t = weirdAngleWrap(wantedAngle) * TurretConstants.ticksPerRadian;
                    double error = t-currentPositionTicks;
                    errorTicks = error;

                    double power = MathUtil.clamp(
                                turretController.calculate(currentPositionTicks, t) +
                                        TurretConstants.kS * Math.signum(error) +
                                        TurretConstants.kV * wantedAngularVelocity +
                                        TurretConstants.kAngularProcession * currentVelocityTicks * flywheelVelocityTicks
                            ,
                                -TurretConstants.maxPower,
                                TurretConstants.maxPower
                    );
                    if (TurretConstants.useVoltageCompensation) {
                        power *= (TurretConstants.nominalVoltage / voltageSensor.getVoltage());
                    }
//                    if (Math.abs(prevSetPower - power) > 0.03) {
//                        turretMotor.setPower(power);
//                        prevSetPower = power;
//                    }
                    turretMotor.setPower(power);
                }
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

    public static double weirdAngleWrap(double radians) {
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
}
