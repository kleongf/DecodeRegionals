package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;

public class PoseConstants {
    // Teleop Poses
    public static double DISTANCE_IN = 10; // if it is not holding/correcting can increase this number
    public static double ROBOT_CENTER_TO_FRONT_LENGTH = 9; // center of drivetrain to front of robot
    public static double ROBOT_WIDTH = 15.1/2d;
    public static Pose BLUE_CORNER_AUTO_POSE = new Pose(27.649656674163825, 132.53866930748939, Math.toRadians(144));
    public static Pose BLUE_GOAL_POSE = new Pose(0, 144, Math.toRadians(135));
    public static Pose RED_GOAL_POSE = new Pose(144, 144, Math.toRadians(45));
    public static Pose BLUE_FAR_POSE =  new Pose(54, 12, Math.toRadians(180));
    public static Pose RED_FAR_POSE =  new Pose(144-54, 12, Math.toRadians(0));
    public static Pose BLUE_GATE_POSE = new Pose(14, 70, Math.toRadians(270));

    public static Pose RED_GATE_POSE = new Pose(144-14, 70, Math.toRadians(270));

    public static Pose RED_PARK_POSE = new Pose(39, 35, Math.toRadians(90));
    public static Pose BLUE_PARK_POSE = new Pose(144-39, 35, Math.toRadians(90));

    // Autonomous Poses
    public static Pose BLUE_CLOSE_AUTO_POSE = new Pose(31.5, 137.6, Math.toRadians(270));
    public static Pose RED_CLOSE_AUTO_POSE = new Pose(144-31.5, 137.6, Math.toRadians(270));
    // TODO: rename to blue far auto start pose
    // TODO: these poses are a bit off. change them to what they should be based on limelight pose test.
    // TODO: also find these poses for far auto, maybe tomorrow if have time
    public static Pose BLUE_FAR_AUTO_POSE = new Pose(55.5, 6.2, Math.toRadians(90));
    public static Pose RED_FAR_AUTO_POSE = new Pose(144-55.5, 6.2, Math.toRadians(90));

    public static Pose BLUE_END_AUTO_POSE = new Pose(54, 115, Math.toRadians(-118));
    public static Pose RED_END_AUTO_POSE = new Pose(144-54, 115, Math.toRadians(180-(-118)));
    public static Pose BLUE_END_FAR_AUTO_POSE = new Pose(36, 12, Math.toRadians(180));
    public static Pose RED_END_FAR_AUTO_POSE = new Pose(144 - 36, 12, Math.toRadians(180-180));
    // 144-12.4

    // TODO: FIND NEW GATE POSITIONS
    // 14.2 60.4, 147
    public static Pose BLUE_FAR_GATE_AUTO_POSE = new Pose(14.4, 60.5, Math.toRadians(148));
    // red not tuned
    public static Pose RED_FAR_GATE_AUTO_POSE = new Pose(144-14.4, 60.5, Math.toRadians(180-148));
    public static Pose BLUE_FAR_GATE_AUTO_POSE_IN = new Pose(BLUE_FAR_GATE_AUTO_POSE.getX()+Math.cos(BLUE_FAR_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_FAR_GATE_AUTO_POSE.getY()+Math.sin(BLUE_FAR_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_FAR_GATE_AUTO_POSE.getHeading());
    public static Pose RED_FAR_GATE_AUTO_POSE_IN = new Pose(RED_FAR_GATE_AUTO_POSE.getX()+Math.cos(RED_FAR_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_FAR_GATE_AUTO_POSE.getY()+Math.sin(RED_FAR_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_FAR_GATE_AUTO_POSE.getHeading());

    public static Pose BLUE_GATE_AUTO_POSE =  new Pose(12.8, 60.7, Math.toRadians(148));
    //public static Pose RED_GATE_AUTO_POSE = new Pose(144-12.4, 60, Math.toRadians(180-148));
    public static Pose RED_GATE_AUTO_POSE = new Pose(129.5, 60, Math.toRadians(180-148));
    public static Pose BLUE_GATE_AUTO_POSE_IN =  new Pose(BLUE_GATE_AUTO_POSE.getX()+Math.cos(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getY()+Math.sin(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE.getHeading());
    public static Pose RED_GATE_AUTO_POSE_IN =  new Pose(RED_GATE_AUTO_POSE.getX()+Math.cos(RED_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_GATE_AUTO_POSE.getY()+Math.sin(RED_GATE_AUTO_POSE.getHeading())*DISTANCE_IN, RED_GATE_AUTO_POSE.getHeading());
    public static Pose BLUE_SHOOT_AUTO_POSE = new Pose(54, 78, Math.toRadians(148));
    public static Pose RED_SHOOT_AUTO_POSE = new Pose(144-54, 78, Math.toRadians(180-148));
}
