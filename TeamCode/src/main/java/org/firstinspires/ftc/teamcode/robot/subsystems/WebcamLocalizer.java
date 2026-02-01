package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.geometry.Pose;

public class WebcamLocalizer {
    // (0, 0) -> (72, 72)
    // pp x is cam y
    // pp y is cam -x

    // (72+y), (72-x)
    public static Pose toPinpointPose(Pose webcamPose) {
        return new Pose(72 + webcamPose.getY(), 72 - webcamPose.getX(), webcamPose.getHeading());
    }
}
