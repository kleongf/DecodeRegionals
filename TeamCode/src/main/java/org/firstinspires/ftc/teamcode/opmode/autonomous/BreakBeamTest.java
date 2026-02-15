package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
 * This OpMode demonstrates how to use a digital channel.
 *
 * The OpMode assumes that the digital channel is configured with a name of "digitalTouch".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Sensor: break beam")
public class BreakBeamTest extends LinearOpMode {
    DigitalChannel digitalTouch;  // Digital channel Object

    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "topSensor");

        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read the digital channel.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // button is pressed if value returned is LOW or false.
            // send the info back to driver station using telemetry function.
            if (digitalTouch.getState() == false) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }

            telemetry.update();
        }
    }
}