package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.decodeutil.Flipper;

public class FieldConstants {
    public static String END_POSE_KEY = "END_POSE";
    public static double DISTANCE_IN = 15; // if it is not holding/correcting can increase this number
    public static double ROBOT_WIDTH = 15.1/2d;
    public static double FIELD_WIDTH = 143;
    public static double ROBOT_BOTTOM_TO_CENTER = 6.14173; // inches
    public static double BLUE_WALL_LEFT_DISTANCE = 47.5; // TODO: measured to be 47.5
    public static Pose BLUE_STANDARD_START_POSE = new Pose(BLUE_WALL_LEFT_DISTANCE + ROBOT_WIDTH, ROBOT_BOTTOM_TO_CENTER, Math.toRadians(90));
    public static Pose RED_STANDARD_START_POSE = Flipper.flip(BLUE_STANDARD_START_POSE);
    public static Pose BLUE_RELOCALIZATION_POSE = new Pose(FIELD_WIDTH - ROBOT_WIDTH, ROBOT_BOTTOM_TO_CENTER, Math.toRadians(90));
    public static Pose RED_RELOCALIZATION_POSE = Flipper.flip(BLUE_RELOCALIZATION_POSE);
    public static Pose BLUE_GOAL_POSE = new Pose(12, FIELD_WIDTH-10, Math.toRadians(135));
    public static Pose RED_GOAL_POSE = Flipper.flip(BLUE_GOAL_POSE);

    public static Pose BLUE_SIDE_GATE_POSE = new Pose(16, 70, Math.toRadians(270));

    public static Pose RED_SIDE_GATE_POSE = Flipper.flip(BLUE_SIDE_GATE_POSE);
    public static Pose RED_PARK_POSE = new Pose(39, 33, Math.toRadians(0));
    public static Pose BLUE_PARK_POSE = Flipper.flip(RED_PARK_POSE);

    // Autonomous Poses
    public static Pose BLUE_CLOSE_AUTO_POSE = new Pose(31.5, 135.8, Math.toRadians(270));
    // TODO: rename to this one
    public static Pose BLUE_CLOSE_AUTO_START_POSE = new Pose(31.5, 135.8, Math.toRadians(270));
    public static Pose RED_CLOSE_AUTO_POSE = Flipper.flip(BLUE_CLOSE_AUTO_POSE);
    public static Pose BLUE_FAR_AUTO_START_POSE = new Pose(50, 8, Math.toRadians(180));
    public static Pose RED_FAR_AUTO_POSE = new Pose(FIELD_WIDTH-42, 8, Math.toRadians(180-180));
    // TODO: rename to blue end close auto pose
    public static Pose BLUE_END_AUTO_POSE = new Pose(58, 96, Math.toRadians(-127));
    public static Pose RED_END_AUTO_POSE = Flipper.flip(BLUE_END_AUTO_POSE);
    public static Pose BLUE_END_FAR_AUTO_POSE = new Pose(36, 12, Math.toRadians(180));
    public static Pose RED_END_FAR_AUTO_POSE = new Pose(FIELD_WIDTH - 36, 12, Math.toRadians(180-180));

    public static Pose BLUE_GATE_AUTO_POSE =  new Pose(14, 59, Math.toRadians(147));
    public static Pose RED_GATE_AUTO_POSE = new Pose(FIELD_WIDTH-14, 59, Math.toRadians(180-147));
    public static Pose BLUE_GATE_AUTO_POSE_IN =  new Pose(BLUE_GATE_AUTO_POSE.getX()+Math.cos(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getY()+Math.sin(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getHeading()+Math.toRadians(4));
    public static Pose RED_GATE_AUTO_POSE_IN =  new Pose(RED_GATE_AUTO_POSE.getX()+Math.cos(RED_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_GATE_AUTO_POSE.getY()+Math.sin(RED_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_GATE_AUTO_POSE.getHeading()-Math.toRadians(4));
}
