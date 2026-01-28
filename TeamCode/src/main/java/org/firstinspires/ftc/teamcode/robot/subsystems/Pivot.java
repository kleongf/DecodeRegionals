package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

public class Pivot extends Subsystem {
    public Servo pivotServo1;
    public Servo pivotServo2;
    public Pivot(HardwareMap hardwareMap) {
        pivotServo1 = hardwareMap.get(Servo.class, "hang1");
        pivotServo2 = hardwareMap.get(Servo.class, "hang2");
        //pivotServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        //pivotServo2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void update() {

    }

    @Override
    public void start() {

    }

    public void setPower(double power) {
        pivotServo1.setPosition((1+power)/2);
        pivotServo2.setPosition(1-(1+power)/2);
    }
}
