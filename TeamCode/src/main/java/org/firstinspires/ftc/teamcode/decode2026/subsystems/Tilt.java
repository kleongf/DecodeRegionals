package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.decode2026.constants.TiltConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;

public class Tilt extends Subsystem {
    public enum Mode {
        TILT_ON,
        TILT_OFF
    }
    public Mode wantedMode;
    private final Servo tiltServo;
    public boolean tilted;
    public Tilt(HardwareMap hardwareMap) {
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        tiltServo.setPosition(TiltConstants.TILT_SERVO_UNTILTED);
        tilted = false;
        wantedMode = Mode.TILT_OFF;
    }

    @Override
    public void reset() {
        wantedMode = Mode.TILT_OFF;
    }

    @Override
    public void start() {
        wantedMode = Mode.TILT_ON;
    }

    @Override
    public void update() {
        super.update();
    }
    public void tilt() {
        tiltServo.setPosition(TiltConstants.TILT_SERVO_TILTED);
        tilted = true;
    }
    public void unTilt() {

        tiltServo.setPosition(TiltConstants.TILT_SERVO_UNTILTED);
        tilted = false;
    }

}

