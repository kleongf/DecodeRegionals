package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.lib.robot.Robot;

public class CurrentRobot extends Robot {
    public final Drivetrain drivetrain;

    public CurrentRobot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void init() {
        drivetrain.init();
    }

    @Override
    public void start() {
        drivetrain.start();
    }
}
