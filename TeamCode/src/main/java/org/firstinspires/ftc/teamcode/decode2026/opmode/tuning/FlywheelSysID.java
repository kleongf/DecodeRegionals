package org.firstinspires.ftc.teamcode.decode2026.opmode.tuning;

import static org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil.multipleLinearRegression;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name="Flywheel System Identification")
public class FlywheelSysID extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private VoltageSensor voltageSensor;
    private double power = 0;
    private double prevVelocity = 0;
    private ElapsedTime elapsedTime;
    private boolean stopped = false;
    // solve for ks, a, and b
    private List<Double> accelerations;
    private List<Double> velocities;
    private List<Double> voltages;
    private double[] coefficients = {0, 0, 0};

    @Override

    public void loop() {
        if (gamepad1.xWasPressed()) {
            stopped = true;
            // perform the regression and stuff
            int m = voltages.size();
            double[][] designMatrix = new double[m][3]; // 3 cols: intercept + 3 vars

            for (int i = 0; i < m; i++) {
                designMatrix[i][0] = 1.0;   // intercept term
                designMatrix[i][1] = velocities.get(i);
                designMatrix[i][2] = accelerations.get(i);
            }
            double[] voltagesArray = voltages.stream()
                    .mapToDouble(Double::doubleValue)
                    .toArray();

            coefficients = multipleLinearRegression(designMatrix, voltagesArray);
        }

        if (!stopped) {
            shooterMotor.setPower(power);
            shooterMotor2.setPower(power);
            double voltage = (voltageSensor.getVoltage() / 12.0) * -power;
            double velocity = shooterMotor.getVelocity();
            // check before adding it to points, cant divide by 0
            if (elapsedTime.seconds() > 1e-6) {
                double acceleration = (velocity - prevVelocity) / elapsedTime.seconds();
                accelerations.add(acceleration);
                velocities.add(velocity);
                voltages.add(voltage);
            }

            prevVelocity = velocity;
            power += 0.003;
            elapsedTime.reset();
        } else {
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
            telemetry.addData("Coefficient kS", coefficients[0]);
            telemetry.addData("Coefficient kV", coefficients[1]);
            telemetry.addData("Coefficient kA", coefficients[2]);
        }

        telemetry.update();
    }

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        accelerations = new ArrayList<>();
        voltages = new ArrayList<>();
        velocities = new ArrayList<>();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }
}
