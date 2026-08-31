package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Tests the pure rotation/blend math in {@link Drivetrain#computeDrivePowers}, in isolation
 * from Follower/HardwareMap, covering the alliance-heading-flip behavior specifically:
 * manual joystick input must flip with alliance, but locked-axis PID output must not.
 */
public class DrivetrainTest {

    private static final double EPS = 1e-6;

    @Test
    public void robotCentricManualDriveIgnoresHeadingAndAlliance() {
        double[] a = Drivetrain.computeDrivePowers(
                1.3, Math.PI, true,
                1, 0, 0.5,
                false, 0, false, 0, false, 0
        );
        double[] b = Drivetrain.computeDrivePowers(
                -2.1, 0, true,
                1, 0, 0.5,
                false, 0, false, 0, false, 0
        );

        assertArrayEquals(new double[] {1, 0, 0.5}, a, EPS);
        assertArrayEquals("robot-centric mode should be alliance/heading-invariant", a, b, EPS);
    }

    @Test
    public void fieldCentricAllianceOffsetFlipsManualDirection() {
        double[] red = Drivetrain.computeDrivePowers(
                0, 0, false,
                1, 0, 0,
                false, 0, false, 0, false, 0
        );
        double[] blue = Drivetrain.computeDrivePowers(
                0, Math.PI, false,
                1, 0, 0,
                false, 0, false, 0, false, 0
        );

        assertEquals(1, red[0], EPS);
        assertEquals(0, red[1], EPS);
        assertEquals(-1, blue[0], EPS);
        assertEquals(0, blue[1], EPS);
    }

    @Test
    public void lockedAxisOutputIsAllianceInvariant() {
        double heading = 0.4;
        double[] red = Drivetrain.computeDrivePowers(
                heading, 0, false,
                0, 0, 0,
                true, 0.7, true, -0.3, false, 0
        );
        double[] blue = Drivetrain.computeDrivePowers(
                heading, Math.PI, false,
                0, 0, 0,
                true, 0.7, true, -0.3, false, 0
        );
        double[] robotCentric = Drivetrain.computeDrivePowers(
                heading, 0, true,
                0, 0, 0,
                true, 0.7, true, -0.3, false, 0
        );

        assertArrayEquals("locked axis output must not depend on alliance offset", red, blue, EPS);
        assertArrayEquals("locked axis output must not depend on the robot-centric toggle", red, robotCentric, EPS);
    }

    @Test
    public void lockedAxisRotatesCorrectlyIntoRobotFrame() {
        double[] powers = Drivetrain.computeDrivePowers(
                Math.PI / 2, 0, false,
                0, 0, 0,
                true, 1, false, 0, false, 0
        );

        assertEquals(0, powers[0], EPS);
        assertEquals(-1, powers[1], EPS);
    }

    @Test
    public void headingLockUsesPidOutputDirectlyIgnoringManualTurn() {
        double[] powers = Drivetrain.computeDrivePowers(
                0, 0, true,
                0, 0, 0.9,
                false, 0, false, 0, true, -0.6
        );

        assertEquals(-0.6, powers[2], EPS);
    }
}
