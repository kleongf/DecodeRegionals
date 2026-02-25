package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.robot.subsystems.ArtifactVision2;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;

@Config
@TeleOp(name="Vision Distance Test Red")
public class VisionDistanceTestRed extends OpMode {
    private ArtifactVision2 vision;
    @Override
    public void loop() {
        vision.update();
        telemetry.addData("best X pos", vision.getBestXMovingAverage());
        telemetry.update();

    }

    @Override
    public void init() {
        vision = new ArtifactVision2(hardwareMap, Alliance.RED);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        vision.start();
    }
}
