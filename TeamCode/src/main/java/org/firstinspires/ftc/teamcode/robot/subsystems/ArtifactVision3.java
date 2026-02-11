package org.firstinspires.ftc.teamcode.robot.subsystems;

public class ArtifactVision3 {
    // new idea:
    // if we can get precise locations of balls very easily
    // and we angle the camera down more (and have a smaller fov camera)
    // we shouldn't prioritize based on size, if a ball is close, just go to it lo

    // although we should prioritize balls to the left
    // keep driving until either 3 balls or at low x value or timeout? nah timeout is bad
    // (or if no target spotted and target x not reached and intake not full, go to corner?)

    // if the target changed by a lot and a ball was not intaken then we should not switch the target
    // i have no idea ngl the method we use will require a lot of testing

    // but balls almost always end up at the wall anyway

    // i just think we should have a centered and slightly angled down camera
}
