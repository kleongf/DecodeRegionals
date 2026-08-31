package org.firstinspires.ftc.teamcode.biobuzz.commands;

import org.firstinspires.ftc.teamcode.biobuzz.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.lib.fsm.Transition;
import org.firstinspires.ftc.teamcode.lib.robot.Command;

import java.util.Collections;

public class TurnToHeadingCommand extends Command {
    public TurnToHeadingCommand(Drivetrain drivetrain, double targetHeadingRadians) {
        super(Collections.singletonList(drivetrain), new StateMachine(
                new State()
                        .onEnter(() -> drivetrain.lockHeading(targetHeadingRadians))
                        .onExit(drivetrain::unlockHeading)
                        .transition(new Transition(() -> drivetrain.atHeadingTarget(DrivetrainConstants.LOCK_TOLERANCE_HEADING)))
                        .maxTime(2000)
        ));
    }
}
