package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.pedropathing.geometry.Pose;

public class FieldConstants {
    public static String END_POSE_KEY = "END_POSE";
    public static double DISTANCE_IN = 15; // if it is not holding/correcting can increase this number
    public static double ROBOT_WIDTH = 15.1/2d;
    public static double FIELD_WIDTH = 142;
    public static double ROBOT_BOTTOM_TO_CENTER = 6.14173; // inches
    public static double BLUE_WALL_LEFT_DISTANCE = 47.5; // TODO: measured to be 47.5
    public static Pose BLUE_STANDARD_START_POSE = new Pose(BLUE_WALL_LEFT_DISTANCE + ROBOT_WIDTH, ROBOT_BOTTOM_TO_CENTER, Math.toRadians(90));
    public static Pose RED_STANDARD_START_POSE = new Pose(FIELD_WIDTH - BLUE_STANDARD_START_POSE.getX(), ROBOT_BOTTOM_TO_CENTER, Math.toRadians(90));
    public static Pose BLUE_RELOCALIZATION_POSE = new Pose(FIELD_WIDTH - ROBOT_WIDTH, ROBOT_BOTTOM_TO_CENTER, Math.toRadians(90));
    public static Pose RED_RELOCALIZATION_POSE = new Pose(ROBOT_WIDTH, ROBOT_BOTTOM_TO_CENTER, Math.toRadians(90));
    public static Pose BLUE_GOAL_POSE = new Pose(12, FIELD_WIDTH-10, Math.toRadians(135));
    public static Pose RED_GOAL_POSE = new Pose(FIELD_WIDTH-12, FIELD_WIDTH-10, Math.toRadians(45));

    public static Pose BLUE_SIDE_GATE_POSE = new Pose(16, 70, Math.toRadians(270));

    public static Pose RED_SIDE_GATE_POSE = new Pose(FIELD_WIDTH-16, 70, Math.toRadians(270));
    // TODO: find new park poses that are good
    public static Pose RED_PARK_POSE = new Pose(30, 30, Math.toRadians(225));
    public static Pose BLUE_PARK_POSE = new Pose(FIELD_WIDTH-30, 30, Math.toRadians(180-225));

    // Autonomous Poses
    public static Pose BLUE_CLOSE_AUTO_POSE = new Pose(31.5, 135.8, Math.toRadians(270));
    public static Pose RED_CLOSE_AUTO_POSE = new Pose(FIELD_WIDTH-31.5, 135.8, Math.toRadians(270));
    public static Pose BLUE_FAR_AUTO_POSE = new Pose(42, 8, Math.toRadians(180));
    public static Pose RED_FAR_AUTO_POSE = new Pose(FIELD_WIDTH-42, 8, Math.toRadians(180-180));

    public static Pose BLUE_END_AUTO_POSE = new Pose(58, 96, Math.toRadians(-127));
    public static Pose RED_END_AUTO_POSE = new Pose(FIELD_WIDTH-58, 96, Math.toRadians(180-(-127)));
    public static Pose BLUE_END_FAR_AUTO_POSE = new Pose(36, 12, Math.toRadians(180));
    public static Pose RED_END_FAR_AUTO_POSE = new Pose(FIELD_WIDTH - 36, 12, Math.toRadians(180-180));

    public static Pose BLUE_GATE_AUTO_POSE =  new Pose(14, 59, Math.toRadians(147));
    public static Pose RED_GATE_AUTO_POSE = new Pose(FIELD_WIDTH-14, 59, Math.toRadians(180-147));
    public static Pose BLUE_GATE_AUTO_POSE_IN =  new Pose(BLUE_GATE_AUTO_POSE.getX()+Math.cos(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getY()+Math.sin(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getHeading()+Math.toRadians(4));
    public static Pose RED_GATE_AUTO_POSE_IN =  new Pose(RED_GATE_AUTO_POSE.getX()+Math.cos(RED_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_GATE_AUTO_POSE.getY()+Math.sin(RED_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_GATE_AUTO_POSE.getHeading()-Math.toRadians(4));
}
