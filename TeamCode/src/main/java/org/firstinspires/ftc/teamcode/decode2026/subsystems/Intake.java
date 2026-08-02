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
    public boolean isOneBall;
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
        double wantedSpeed = 0;
        switch (wantedMode) {
            case INTAKE_FAST:
                wantedSpeed = IntakeConstants.INTAKE_FAST_POWER;
                break;
            case INTAKE_MEDIUM:
                wantedSpeed = IntakeConstants.INTAKE_MEDIUM_POWER;
                break;
            case INTAKE_SLOW:
                wantedSpeed = IntakeConstants.INTAKE_SLOW_POWER;
                break;
            case INTAKE_OFF:
                wantedSpeed = IntakeConstants.INTAKE_STOPPED_POWER;
                break;
            case INTAKE_BACKWARD:
                wantedSpeed = IntakeConstants.INTAKE_BACKWARD_POWER;
                break;
        }
        if (!IntakeConstants.useMotorCaching || Math.abs(prevSetPower - wantedSpeed) > IntakeConstants.cachingThreshold) {
            intakeMotor.setPower(wantedSpeed);
            prevSetPower = wantedSpeed;
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
        isOneBall = detectionState == DetectionState.FIRST_TRIGGERED;
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
