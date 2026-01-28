package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@Config
@TeleOp(name="Intake Full Test")
public class IntakeFullTest extends OpMode {
    private Intake intake;

    @Override
    public void loop() {
        intake.update();
        telemetry.addData("Is Full:", intake.intakeFull());
        telemetry.addData("Bottom Beam", intake.getBottomBeam());
        telemetry.addData("Middle Beam", intake.getMiddleBeam());
        telemetry.addData("Top Beam", intake.getTopBeam());
        telemetry.update();

    }

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        intake.start();
    }
}
