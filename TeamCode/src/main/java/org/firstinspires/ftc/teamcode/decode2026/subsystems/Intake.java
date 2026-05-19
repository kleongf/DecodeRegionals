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
        INTAKE_SLOW,
        INTAKE_OFF
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
    private final DcMotorEx intakeMotor;
    private final DigitalChannel top, middle, bottom;

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
                intakeMotor.setPower(IntakeConstants.INTAKE_FAST_POWER);
                break;
            case INTAKE_SLOW:
                intakeMotor.setPower(IntakeConstants.INTAKE_SLOW_POWER);
                break;
            case INTAKE_OFF:
                intakeMotor.setPower(IntakeConstants.INTAKE_STOPPED_POWER);
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
