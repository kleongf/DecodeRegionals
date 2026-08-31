package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.decode2026.constants.LEDIndicatorConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;

public class LEDIndicator extends Subsystem {
    public enum Mode {
        INTAKE_FULL,
        RELOCALIZING,
        IDLE
    }
    public Mode wantedMode;
    private final ElapsedTime timer;
    private final ElapsedTime flashingTimer;
    private final Servo light;

    public LEDIndicator(HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        flashingTimer = new ElapsedTime();
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(LEDIndicatorConstants.minBrightness);
    }

    @Override
    public void reset() {
        wantedMode = Mode.IDLE;
        timer.reset();
        flashingTimer.reset();
    }

    @Override
    public void start() {
        wantedMode = Mode.IDLE;
    }

    @Override
    public void update() {
        switch (wantedMode) {
            case INTAKE_FULL:
                if (flashingTimer.seconds() < LEDIndicatorConstants.flashDuration) {
                    if (Math.round(flashingTimer.seconds() * 10) % 2 == 0) {
                        light.setPosition(LEDIndicatorConstants.maxBrightness);
                    } else {
                        light.setPosition(LEDIndicatorConstants.minBrightness);
                    }
                } else {
                    wantedMode = Mode.IDLE;
                }
                break;
            case RELOCALIZING:
                if (timer.seconds() < LEDIndicatorConstants.flashDuration) {
                    light.setPosition(LEDIndicatorConstants.maxBrightness);
                } else {
                    wantedMode = Mode.IDLE;
                }
                break;
            case IDLE:
                light.setPosition(LEDIndicatorConstants.minBrightness);
                break;
        }
    }

    public void indicateRelocalization() {
        wantedMode = Mode.RELOCALIZING;
        timer.reset();
    }

    public void indicateIntakeFull() {
        wantedMode = Mode.INTAKE_FULL;
        flashingTimer.reset();
    }
}

