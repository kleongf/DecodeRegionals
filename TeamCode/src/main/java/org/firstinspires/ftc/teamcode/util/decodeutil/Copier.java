package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;

public class Copier {
    // copies the first path in a PathChain and interpolates linearly.
    public static PathChain copy(Follower follower, PathChain pathChain) {
        Path firstPath = pathChain.firstPath();
        if (firstPath.getControlPoints().size() <= 2) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(
                            firstPath.getFirstControlPoint(),
                            firstPath.getLastControlPoint()
                    ))
                    .setLinearHeadingInterpolation(firstPath.getHeadingGoal(0), firstPath.getHeadingGoal(1))
                    .build();
        }
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        firstPath.getControlPoints()
                ))
                .setLinearHeadingInterpolation(firstPath.getHeadingGoal(0), firstPath.getHeadingGoal(1))
                .build();
    }
}
