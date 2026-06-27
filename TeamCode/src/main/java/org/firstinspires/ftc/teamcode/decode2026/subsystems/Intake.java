package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.decode2026.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;

public class Intake extends Subsystem {
    public enum Mode {
        INTAKE_FAST,
        INTAKE_MEDIUM,
        INTAKE_SLOW,
        INTAKE_OFF,
        INTAKE_BACKWARD,
    }
    public enum DetectionState {
        EMPTY,
        FIRST_TRIGGERED,
        SECOND_TRIGGERED,
        THIRD_TRIGGERED
    }
    public Mode wantedMode;
    public DetectionState detectionState;
    public boolean isFull;
    public boolean isMostlyFull;
    private final DcMotorEx intakeMotor;
    private final DigitalChannel top, middle, bottom;
    private double prevSetPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        top = hardwareMap.get(DigitalChannel.class, "topSensor");
        middle = hardwareMap.get(DigitalChannel.class, "middleSensor");
        bottom = hardwareMap.get(DigitalChannel.class, "bottomSensor");

        top.setMode(DigitalChannel.Mode.INPUT);
        middle.setMode(DigitalChannel.Mode.INPUT);
        bottom.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void start() {
        wantedMode = Mode.INTAKE_FAST;
    }

    @Override
    public void reset() {
        wantedMode = Mode.INTAKE_OFF;
        detectionState = DetectionState.EMPTY;
    }

    @Override
    public void update() {
        switch (wantedMode) {
            case INTAKE_FAST:
                // caching
                if (Math.abs(prevSetPower - IntakeConstants.INTAKE_FAST_POWER) > 0.03) {
                    intakeMotor.setPower(IntakeConstants.INTAKE_FAST_POWER);
                    prevSetPower = IntakeConstants.INTAKE_FAST_POWER;
                }
                // intakeMotor.setPower(IntakeConstants.INTAKE_FAST_POWER);
                break;
            case INTAKE_MEDIUM:
                if (Math.abs(prevSetPower - IntakeConstants.INTAKE_MEDIUM_POWER) > 0.03) {
                    intakeMotor.setPower(IntakeConstants.INTAKE_MEDIUM_POWER);
                    prevSetPower = IntakeConstants.INTAKE_MEDIUM_POWER;
                }
                // intakeMotor.setPower(IntakeConstants.INTAKE_MEDIUM_POWER);
                break;
            case INTAKE_SLOW:
                if (Math.abs(prevSetPower - IntakeConstants.INTAKE_SLOW_POWER) > 0.03) {
                    intakeMotor.setPower(IntakeConstants.INTAKE_SLOW_POWER);
                    prevSetPower = IntakeConstants.INTAKE_SLOW_POWER;
                }
                // intakeMotor.setPower(IntakeConstants.INTAKE_SLOW_POWER);
                break;
            case INTAKE_OFF:
                if (Math.abs(prevSetPower - IntakeConstants.INTAKE_STOPPED_POWER) > 0.03) {
                    intakeMotor.setPower(IntakeConstants.INTAKE_STOPPED_POWER);
                    prevSetPower = IntakeConstants.INTAKE_STOPPED_POWER;
                }
                // intakeMotor.setPower(IntakeConstants.INTAKE_STOPPED_POWER);
                break;
            case INTAKE_BACKWARD:
                if (Math.abs(prevSetPower - IntakeConstants.INTAKE_BACKWARD_POWER) > 0.03) {
                    intakeMotor.setPower(IntakeConstants.INTAKE_BACKWARD_POWER);
                    prevSetPower = IntakeConstants.INTAKE_BACKWARD_POWER;
                }
                break;
        }

        switch (detectionState) {
            case EMPTY:
                if (topTriggered()) {
                    detectionState = DetectionState.FIRST_TRIGGERED;
                }
                break;
            case FIRST_TRIGGERED:
                if (middleTriggered()) {
                    detectionState = DetectionState.SECOND_TRIGGERED;
                }
                break;
            case SECOND_TRIGGERED:
                if (bottomTriggered()) {
                    detectionState = DetectionState.THIRD_TRIGGERED;
                }
                break;
            case THIRD_TRIGGERED:
                break;
        }
        isMostlyFull = detectionState == DetectionState.SECOND_TRIGGERED;
        isFull = detectionState == DetectionState.THIRD_TRIGGERED;
    }

    public boolean topTriggered() {
        return !top.getState();
    }

    public boolean middleTriggered() {
        return !middle.getState();
    }

    public boolean bottomTriggered() {
        return !bottom.getState();
    }
}
