package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

public class Pivot extends Subsystem {
    public Servo hang1;
    public Pivot(HardwareMap hardwareMap) {
        hang1 = hardwareMap.get(Servo.class, "hang1");
        // unTilt();
    }

    @Override
    public void update() {

    }

    @Override
    public void start() {

    }

    public void tilt() {
        hang1.setPosition(RobotConstants.TILT_SERVO_TILTED);
    }

    public void unTilt() {
        hang1.setPosition(RobotConstants.TILT_SERVO_UNTILTED);
    }
}
