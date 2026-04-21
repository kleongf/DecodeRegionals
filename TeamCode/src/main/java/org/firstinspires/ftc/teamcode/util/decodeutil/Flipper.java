package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;

public class Flipper {
    public static Pose flip(Pose pose) {
        return new Pose(FieldConstants.FIELD_WIDTH - pose.getX(), pose.getY(), Math.toRadians(180) - pose.getHeading());
    }
}
