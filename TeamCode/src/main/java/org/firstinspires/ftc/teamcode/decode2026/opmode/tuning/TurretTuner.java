package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;
@Config
@TeleOp(name="Turret PIDF Tuner")
public class TurretTuner extends OpMode {
    private Turret turret;
    public static double kP = 0.004;
    public static double kD = 0.00000;
    public static double target = 0;
    private double ticksPerRevolution = 1381; // 383.6*5
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    @Override
    public void loop() {
        turret.setPDCoefficients(kP, kD);
        turret.wantedAngle = target;

        turret.update();
        telemetry.addData("current pos", turret.currentPositionTicks);
        telemetry.addData("target pos", turret.weirdAngleWrap(Math.toRadians(target)) * ticksPerRadian);
        telemetry.update();

    }

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        turret.start();
    }
}
