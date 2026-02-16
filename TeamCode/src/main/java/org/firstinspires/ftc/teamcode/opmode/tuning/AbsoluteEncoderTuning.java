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
    public static double encoderOffsetDegrees = -120;
    public static double minVoltage = 0.02;
    public static double maxVoltage = 3.29; // my understanding: somewhere in the vicinity of 3.2 and 3.3 but depends. is 3.24
    public static boolean isReversed = false;
    private Turret turret;
    private double ticksPerRevolution = 1381; // 383.6*5
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    private double degreesPerTick = 360d / ticksPerRevolution;
    private double prevPos = 0;
    private double encoderGearRatio = 17/14d;
    private AnalogInput encoder;
    private double rotations = 0;

    // idea: convert to radians -> ticks first before encoder gear ratio, then multiply ticks

    // idea 2: use turret encoder: "roughly equals" to determine an actual encoder position

    // this next idea is only really applicable when the absolute encoder reads zero or 2 radians.
    // this should make it so that we don't accidentally turn the wrong direction

    // if motor encoder position is big negative, then we get converted position ticks and subtract 1381
    // if motor encoder position is not big negative, then the converted position is fine

    // logic: if (abs encoder is reading a small negative number): if turret encoder reads a big negative number: subtract 1381 else do nothing
    // because turret encoder is never very far off from the truth

    // logic 2, in case motor slips too much:
    // all turret values will be negative, that's a fact

    // if current - prev is large positive number (say 1000 idk), then current is small neg and prev was big negative
    // meaning the turret must have turned right, which is the positive direction
    // therefore we add one rotation

    // if current - prev is a large negative number (say -1000) then current is big neg and prev was small neg
    // the turret must have turned left, which is the negative direction
    // therefore we subtract one rotation

    private double calculatePositionTicks(double voltage) {
        // the problem might be that the min voltage is 0.02, which is kinda a problem.
        //
         // 1 Software fix: I think the correct formula for calculating your encoder in degrees is (getVoltage() / 3.3) * 360 - offset; with your offset being the amount of degrees from zero. if you want extra precision sometimes the control hub outputs smaller voltages from the actual encoder so your minimum and maximum voltage could be like 0.024 - 3.19

        double directionFactor = isReversed ? -1 : 1;
        double realOffset = Math.toRadians(encoderOffsetDegrees + 360); // added to final

        // if it's reversed
        // new logic: maxVoltage subtraction so that it work. and its positive no matter what
        double position = ((Math.max(0, voltage-minVoltage) /  maxVoltage) * 2 * Math.PI) % (2 * Math.PI) + realOffset;
        double posToTicksGeared = directionFactor * position * ticksPerRadian * encoderGearRatio;

        return posToTicksGeared - 2 * ticksPerRevolution; // + ticksPerRevolution * rotations;


        // double offset = -123.4 + 360;
        // offsetPosition = (encoder.getVoltage() / 3.2 * 360 + offset) % 360;
        // double directionFactor = isReversed ? -1 : 1;
        // we need to convert the offset from ticks to degrees, then turn to radians: 1 tick = 360/1381 degrees.

//        double anglePreGear = MathUtil.normalizeAngle(directionFactor * (voltage / maxVoltage) * 2 * Math.PI);
//        double encoderGearRatio = goofyScaleFactor/14d;
//        double anglePostGear = anglePreGear * encoderGearRatio;
//
//        double angleWithOffset = anglePostGear + realOffset;
//        return turret.weirdAngleWrap(angleWithOffset) * ticksPerRadian;

        // return turret.weirdAngleWrap(directionFactor * (voltage / maxVoltage) * (encoderGearRatio) * 2 * Math.PI + realOffset) * ticksPerRadian;
    }

    // TODO: we probably want to scale voltage in a way where we subtract 0.02 from the voltage output so we can get zero

    private double externalToMotorTicks(double externalTicks, double internalTicks) {
        double smallNegativeThreshold = -200; // don't want to do this for every value
        double largeNegativeThreshold = -1100; // because it skips we can't be perfectly precise but general loc is known

        // if we read a value close to zero but it's closer to negative rotation
        if (externalTicks > smallNegativeThreshold && internalTicks <= largeNegativeThreshold) {
            return externalTicks - ticksPerRevolution;
        }
        // if we read a large negative value but it's actually closer to zero
        if (externalTicks < largeNegativeThreshold && internalTicks >= smallNegativeThreshold) {
            return externalTicks + ticksPerRevolution;
        }

        return externalTicks;
    }


    @Override
    public void loop() {
        double realOffset = Math.toRadians(encoderOffsetDegrees + 360);
        double directionFactor = isReversed ? -1 : 1;
        double voltage = encoder.getVoltage();

        double pos = calculatePositionTicks(voltage);

        // if current - prev is large positive number (say 1000 idk), then current is small neg and prev was big negative
        // meaning the turret must have turned right, which is the positive direction
        // therefore we add one rotation

        // if current - prev is a large negative number (say -1000) then current is big neg and prev was small neg
        // the turret must have turned left, which is the negative direction
        // therefore we subtract one rotation

        double deltaPos = pos - prevPos;

        if (deltaPos > 800) {
            rotations++;
        }

        if (deltaPos < -800) {
            rotations--;
        }

        double rawPos = ((voltage /  maxVoltage) * 2 * Math.PI + realOffset) % (2 * Math.PI);

        prevPos = pos;

        turret.update();
        telemetry.addLine("Turn the turret to 1381 ticks (or -1381 ticks i forgot) to make one revolution, and observe the external encoder conversion. they should be similar.");
        telemetry.addLine("Also, replace the maxVoltage value with the largest voltage you can get by turning the turret (as far as possible). You may repeat multiple times and average them");
        telemetry.addLine("If there is a physical discrepancy between the encoder value and external encoder value, you can add an offset in ticks until it is correct.");
        telemetry.addLine("Tune this until the encoder position and the external encoder position are the same. also restart program if turret position is just off.");
        telemetry.addLine("If the positions are moving in opposite directions then check the box saying 'isReversed' and keep going.");
        telemetry.addData("Turret encoder position", turret.turretMotor.getCurrentPosition());
        telemetry.addData("External encoder position", pos);
        telemetry.addData("External encoder position adjusted", externalToMotorTicks(pos, turret.turretMotor.getCurrentPosition()));
        telemetry.addData("External encoder voltage", voltage);
        telemetry.addData("angle pre gear ratio", rawPos);
        telemetry.addData("angle post gear ratio", rawPos * directionFactor * ticksPerRadian * encoderGearRatio);
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
