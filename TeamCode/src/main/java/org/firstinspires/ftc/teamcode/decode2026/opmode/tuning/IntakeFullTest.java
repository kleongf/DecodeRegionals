package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;

@Config
@TeleOp(name="Intake Full Test", group="?")
public class IntakeFullTest extends OpMode {
    private Intake intake;

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            intake.detectionState = Intake.DetectionState.EMPTY;
        }
        intake.update();
        telemetry.addLine("Press X to reset detection state");
        telemetry.addData("Is Full:", intake.isFull);
        telemetry.addData("Bottom Beam", intake.bottomTriggered());
        telemetry.addData("Middle Beam", intake.middleTriggered());
        telemetry.addData("Top Beam", intake.topTriggered());
        telemetry.update();

    }

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        intake.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        intake.start();
    }
}
