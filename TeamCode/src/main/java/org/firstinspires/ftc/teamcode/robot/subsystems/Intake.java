package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;

import java.util.ArrayList;

public class Intake extends Subsystem {
    public ArrayList<Double> rollingCurrents;
    public ElapsedTime startTimer;
    public enum IntakeState {
        INTAKE_FAST,
        INTAKE_SLOW,
        INTAKE_OFF
    }
    public double CURRENT_LIMIT = 6.0; // 6 amps

    public IntakeState state = IntakeState.INTAKE_OFF;
    public DcMotorEx intakeMotor;
    private DigitalChannel top, middle, bottom;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        top = hardwareMap.get(DigitalChannel.class, "topSensor");
//        middle = hardwareMap.get(DigitalChannel.class, "middleSensor");
//        bottom = hardwareMap.get(DigitalChannel.class, "bottomSensor");
//        top.setMode(DigitalChannel.Mode.INPUT);
//        middle.setMode(DigitalChannel.Mode.INPUT);
//        bottom.setMode(DigitalChannel.Mode.INPUT);

        startTimer = new ElapsedTime();
        rollingCurrents = new ArrayList<>();
    }

    @Override
    public void update() {
        if (rollingCurrents.size() >= 7) {
            rollingCurrents.add(intakeMotor.getCurrent(CurrentUnit.AMPS));
            rollingCurrents.remove(0);
        } else {
            rollingCurrents.add(intakeMotor.getCurrent(CurrentUnit.AMPS));
        }
        switch (state) {
            case INTAKE_FAST:
                intakeMotor.setPower(1);
                break;
            case INTAKE_SLOW:
                intakeMotor.setPower(0.75);
                break;
            case INTAKE_OFF:
                intakeMotor.setPower(0);
                break;
        }
    }

    public boolean stalling() {return intakeMotor.getCurrent(CurrentUnit.AMPS) > CURRENT_LIMIT;}

    @Override
    public void start() {
        startTimer.reset();
    }
    // TODO: DO WHEN WE GET THE THE SENSOR
    public boolean intakeFull() {
        // return false;
        // return !top.getState() && !middle.getState() && !bottom.getState();
        // ah yes don't you just love my naming conventions?
        // we are checking this first, because there has to be a certain number of currents first

        if (rollingCurrents.size() >= 6) {
            boolean rollingCurrentIsHigh = rollingCurrents.stream().allMatch(n -> n > CURRENT_LIMIT);
            return rollingCurrentIsHigh;
        }
        return false;
    }
    // TODO:

    public boolean getBottomBeam() {
        return false;
    }

    public boolean getMiddleBeam() {
        return false;
    }

    public boolean getTopBeam() {
        // !top.getState()
        return false;
    }
}