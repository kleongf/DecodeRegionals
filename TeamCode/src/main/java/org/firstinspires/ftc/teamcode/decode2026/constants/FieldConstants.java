package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.decodeutil.Flipper;

public class FieldConstants {
    public static String END_POSE_KEY = "END_POSE";
    public static double DISTANCE_IN = 15; // if it is not holding/correcting can increase this number
    public static double TURN_IN = Math.toRadians(4); // how much to turn into the gate
    public static double ROBOT_WIDTH = 15.1;
    public static double ROBOT_BUMPER_WIDTH = 16d; // TODO: measure
    public static double HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2;
    public static double ROBOT_BACK_TO_CENTER_DISTANCE = 6.14173; // inches
    public static double GOAL_TO_WALL_DISTANCE = 0.4;
    public static double TILE_WIDTH = 23.2; // TODO: measure
    public static double TILE_TEETH_WIDTH = 0.75; // TODO: measure
    public static double FIELD_WIDTH = 6 * TILE_WIDTH + 4 * TILE_TEETH_WIDTH;
    public static double BLUE_WALL_LEFT_DISTANCE = 2 * TILE_WIDTH + 2 * TILE_TEETH_WIDTH;
    public static Pose BLUE_STANDARD_START_POSE = new Pose(BLUE_WALL_LEFT_DISTANCE + HALF_ROBOT_WIDTH, ROBOT_BACK_TO_CENTER_DISTANCE, Math.toRadians(90));
    public static Pose RED_STANDARD_START_POSE = Flipper.flip(BLUE_STANDARD_START_POSE);
    public static Pose BLUE_RELOCALIZATION_POSE = new Pose(FIELD_WIDTH - HALF_ROBOT_WIDTH, ROBOT_BACK_TO_CENTER_DISTANCE, Math.toRadians(90));
    public static Pose RED_RELOCALIZATION_POSE = Flipper.flip(BLUE_RELOCALIZATION_POSE);
    public static Pose BLUE_GOAL_POSE = new Pose(12, FIELD_WIDTH-10);
    public static Pose RED_GOAL_POSE = Flipper.flip(BLUE_GOAL_POSE);

    public static Pose BLUE_SIDE_GATE_POSE = new Pose(16, 70, Math.toRadians(270));

    public static Pose RED_SIDE_GATE_POSE = Flipper.flip(BLUE_SIDE_GATE_POSE);
    public static Pose BLUE_FRONT_GATE_POSE = new Pose(16, 70, Math.toRadians(180)); // TODO: tune
    public static Pose RED_FRONT_GATE_POSE = Flipper.flip(BLUE_FRONT_GATE_POSE);
    public static Pose RED_PARK_POSE = new Pose(39, 33, Math.toRadians(0));
    public static Pose BLUE_PARK_POSE = Flipper.flip(RED_PARK_POSE);

    // Autonomous Poses
    public static Pose BLUE_CLOSE_START_AUTO_POSE = new Pose(TILE_WIDTH + TILE_TEETH_WIDTH + HALF_ROBOT_WIDTH, FIELD_WIDTH - GOAL_TO_WALL_DISTANCE - ROBOT_BACK_TO_CENTER_DISTANCE, Math.toRadians(270));
    public static Pose RED_CLOSE_START_AUTO_POSE = Flipper.flip(BLUE_CLOSE_START_AUTO_POSE);
    public static Pose BLUE_FAR_START_AUTO_POSE = new Pose(2 * TILE_WIDTH + 2 * TILE_TEETH_WIDTH - ROBOT_BACK_TO_CENTER_DISTANCE, ROBOT_BUMPER_WIDTH / 2d, Math.toRadians(180));
    public static Pose RED_FAR_START_AUTO_POSE = Flipper.flip(BLUE_FAR_START_AUTO_POSE);
    public static Pose BLUE_GATE_AUTO_POSE = new Pose(14, 59, Math.toRadians(147));
    public static Pose RED_GATE_AUTO_POSE = Flipper.flip(BLUE_GATE_AUTO_POSE);
    public static Pose BLUE_GATE_AUTO_POSE_IN = new Pose(BLUE_GATE_AUTO_POSE.getX()+Math.cos(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getY()+Math.sin(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getHeading()+TURN_IN);
    public static Pose RED_GATE_AUTO_POSE_IN = Flipper.flip(BLUE_GATE_AUTO_POSE_IN);
}
