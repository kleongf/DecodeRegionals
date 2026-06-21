package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.decodeutil.Flipper;

public class FieldConstants {
//            12, 60, 142
//                    16, 70
    public static String END_POSE_KEY = "END_POSE";
    // was 15
    public static double DISTANCE_IN = 0; // if it is not holding/correcting can increase this number
    public static double TURN_IN = Math.toRadians(0); // how much to turn into the gate
    public static double ROBOT_WIDTH = 15.1;
    // just counting the drivetrain, without we are 17.0866
    public static double ROBOT_LENGTH = 17.0866;
    public static double ROBOT_EFFECTIVE_LENGTH = 13.5111314961;
    public static double HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2;
    public static double ROBOT_BACK_TO_CENTER_DISTANCE = 6.14173; // inches
    public static double GOAL_TO_WALL_DISTANCE = 0.4;
    public static double FIELD_WIDTH = 142d;
    public static double BLUE_WALL_LEFT_DISTANCE = FIELD_WIDTH / 3;
    public static Pose BLUE_STANDARD_START_POSE = new Pose(BLUE_WALL_LEFT_DISTANCE + HALF_ROBOT_WIDTH, ROBOT_BACK_TO_CENTER_DISTANCE, Math.toRadians(90));
    public static Pose RED_STANDARD_START_POSE = Flipper.flip(BLUE_STANDARD_START_POSE);
    public static Pose BLUE_RELOCALIZATION_POSE = new Pose(FIELD_WIDTH - (ROBOT_LENGTH - ROBOT_BACK_TO_CENTER_DISTANCE), ROBOT_WIDTH, Math.toRadians(0));
    public static Pose RED_RELOCALIZATION_POSE = Flipper.flip(BLUE_RELOCALIZATION_POSE);
    public static Pose BLUE_GOAL_POSE = new Pose(8.4597, 132.2553); // center of mass of goal
            // new Pose(6, FIELD_WIDTH-7);
    public static Pose RED_GOAL_POSE = Flipper.flip(BLUE_GOAL_POSE);

    public static Pose BLUE_SIDE_GATE_POSE = new Pose(16, 70, Math.toRadians(270));

    public static Pose RED_SIDE_GATE_POSE = Flipper.flip(BLUE_SIDE_GATE_POSE);
    public static Pose BLUE_FRONT_GATE_POSE = new Pose(16, 70, Math.toRadians(180)); // TODO: tune
    public static Pose RED_FRONT_GATE_POSE = Flipper.flip(BLUE_FRONT_GATE_POSE);
    public static Pose RED_PARK_POSE = new Pose(30, 24, Math.toRadians(225));
    public static Pose BLUE_PARK_POSE = Flipper.flip(RED_PARK_POSE);


    // Autonomous Poses
    public static Pose BLUE_CLOSE_START_AUTO_POSE = new Pose(FIELD_WIDTH / 6 + HALF_ROBOT_WIDTH, FIELD_WIDTH - GOAL_TO_WALL_DISTANCE - ROBOT_BACK_TO_CENTER_DISTANCE, Math.toRadians(270));
    public static Pose RED_CLOSE_START_AUTO_POSE = Flipper.flip(BLUE_CLOSE_START_AUTO_POSE);
    public static Pose BLUE_FAR_START_AUTO_POSE = new Pose(FIELD_WIDTH / 3 - ROBOT_BACK_TO_CENTER_DISTANCE + 1.0, HALF_ROBOT_WIDTH, Math.toRadians(180));
    public static Pose RED_FAR_START_AUTO_POSE = Flipper.flip(BLUE_FAR_START_AUTO_POSE);
    public static Pose BLUE_GATE_AUTO_POSE_24 = new Pose(12+1.5, 59.5, Math.toRadians(147+3+5+3));
    public static Pose BLUE_GATE_AUTO_POSE_27 = new Pose(12-.3, 59.5+1.2-1, Math.toRadians(147+3+5+3));
    public static Pose RED_GATE_AUTO_POSE_24 = new Pose(FIELD_WIDTH - BLUE_GATE_AUTO_POSE_27.getX()-2+.5+.2, BLUE_GATE_AUTO_POSE_27.getY() - 1-.3, Flipper.flipAngle(BLUE_GATE_AUTO_POSE_27.getHeading())+Math.toRadians(8));
    public static Pose RED_GATE_AUTO_POSE_27 = new Pose(128.5+.5+.5+.5,58.9-1+.5, Math.toRadians(30.5));
    //public static Pose RED_GATE_AUTO_POSE_27 = new Pose(FIELD_WIDTH - BLUE_GATE_AUTO_POSE_27.getX()-2-.5, BLUE_GATE_AUTO_POSE_27.getY() - 1, Flipper.flipAngle(BLUE_GATE_AUTO_POSE_27.getHeading()));
    //public static Pose RED_GATE_AUTO_POSE_27 = new Pose(FIELD_WIDTH - BLUE_GATE_AUTO_POSE_27.getX()-2-.5+1.3+2+.75, BLUE_GATE_AUTO_POSE_27.getY() - 1-.75-.5, Flipper.flipAngle(BLUE_GATE_AUTO_POSE_27.getHeading())+Math.toRadians(8+2));
            // Flipper.flip(BLUE_GATE_AUTO_POSE_27);
    // Math.sin(BLUE_GATE_AUTO_POSE.getHeading())*DISTANCE_IN
    public static Pose BLUE_GATE_AUTO_POSE_IN = new Pose(BLUE_GATE_AUTO_POSE_24.getX()+Math.cos(BLUE_GATE_AUTO_POSE_24.getHeading())*DISTANCE_IN, BLUE_GATE_AUTO_POSE_24.getY(), BLUE_GATE_AUTO_POSE_24.getHeading()+TURN_IN);
    public static Pose RED_GATE_AUTO_POSE_IN = Flipper.flip(BLUE_GATE_AUTO_POSE_IN);

    public static Pose BLUE_CLOSE_ZONE_POSE = new Pose(55, 76);
    public static Pose RED_CLOSE_ZONE_POSE = Flipper.flip(BLUE_CLOSE_ZONE_POSE);
    public static Pose BLUE_FAR_ZONE_POSE = new Pose(60, 12);
    public static Pose RED_FAR_ZONE_POSE = Flipper.flip(BLUE_FAR_ZONE_POSE);
}
