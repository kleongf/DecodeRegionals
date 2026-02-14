package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
@Config
@TeleOp(name="Absolute Encoder Tuner")
public class AbsoluteEncoderTuning extends OpMode {
    // TODO: i also have no idea if it's reversed or not. might need to negate to calculate position
    public static double encoderOffsetDegrees = 0;
    public static double maxVoltage = 3.239; // my understanding: somewhere in the vicinity of 3.2 and 3.3 but depends. is 3.24
    public static boolean isReversed = false;
    private Turret turret;
    private double ticksPerRevolution = 1381; // 383.6*5
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    private double degreesPerTick = 360d / ticksPerRevolution;
    public static double goofyScaleFactor = 17;
    private double encoderGearRatio = goofyScaleFactor/14d;
    private AnalogInput encoder;

    private double calculatePositionTicks(double voltage) {
        double realOffset = Math.toRadians(encoderOffsetDegrees + 360);
        // double offset = -123.4 + 360;
        // offsetPosition = (encoder.getVoltage() / 3.2 * 360 + offset) % 360;
        double directionFactor = isReversed ? -1 : 1;
        // we need to convert the offset from ticks to degrees, then turn to radians: 1 tick = 360/1381 degrees.

        double anglePreGear = MathUtil.normalizeAngle(directionFactor * (voltage / maxVoltage) * 2 * Math.PI);
        double encoderGearRatio = goofyScaleFactor/14d;
        double anglePostGear = anglePreGear * encoderGearRatio;

        double angleWithOffset = anglePostGear + realOffset;
        return turret.weirdAngleWrap(angleWithOffset) * ticksPerRadian;

        // return turret.weirdAngleWrap(directionFactor * (voltage / maxVoltage) * (encoderGearRatio) * 2 * Math.PI + realOffset) * ticksPerRadian;
    }

    @Override
    public void loop() {
        double realOffset = Math.toRadians(encoderOffsetDegrees + 360);
        double directionFactor = isReversed ? -1 : 1;
        double voltage = encoder.getVoltage();
        turret.update();
        telemetry.addLine("Turn the turret to 1381 ticks (or -1381 ticks i forgot) to make one revolution, and observe the external encoder conversion. they should be similar.");
        telemetry.addLine("Also, replace the maxVoltage value with the largest voltage you can get by turning the turret (as far as possible). You may repeat multiple times and average them");
        telemetry.addLine("If there is a physical discrepancy between the encoder value and external encoder value, you can add an offset in ticks until it is correct.");
        telemetry.addLine("Tune this until the encoder position and the external encoder position are the same. also restart program if turret position is just off.");
        telemetry.addLine("If the positions are moving in opposite directions then check the box saying 'isReversed' and keep going.");
        telemetry.addData("Turret encoder position", turret.turretMotor.getCurrentPosition());
        telemetry.addData("External encoder position", calculatePositionTicks(voltage));
        telemetry.addData("External encoder voltage", voltage);
        telemetry.addData("angle pre gear ratio", MathUtil.angleWrap(directionFactor * (voltage / maxVoltage) * 2 * Math.PI));
        telemetry.addData("angle post gear ratio", MathUtil.angleWrap(directionFactor * (voltage / maxVoltage) * 2 * Math.PI) * encoderGearRatio);
        telemetry.addData("angle wrapped weird and offset", turret.weirdAngleWrap(directionFactor * (voltage / maxVoltage) * (encoderGearRatio) * 2 * Math.PI + realOffset));
        telemetry.update();
    }

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.setPDCoefficients(0, 0);
        turret.setFeedforward(0);
        turret.resetEncoder();
        encoder = hardwareMap.get(AnalogInput.class, "externalEncoder");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        turret.start();
    }
}
