package org.firstinspires.ftc.teamcode.lib.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.lib.fsm.Transition;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class CommandSchedulerTest {

    static class FakeSubsystem extends Subsystem {
        int periodicCalls = 0;

        @Override
        public void periodic() {
            periodicCalls++;
        }
    }

    /** A Command whose single-state StateMachine finishes exactly when {@code finished} is set true. */
    static class FakeCommand {
        final Command command;
        final AtomicInteger enters = new AtomicInteger();
        final AtomicInteger exits = new AtomicInteger();
        final AtomicBoolean finished = new AtomicBoolean(false);

        FakeCommand(Subsystem... requirements) {
            StateMachine sm = new StateMachine(
                    new State()
                            .onEnter(enters::incrementAndGet)
                            .onExit(exits::incrementAndGet)
                            .transition(new Transition(finished::get))
            );
            command = new Command(Arrays.asList(requirements), sm);
        }
    }

    @Before
    public void resetScheduler() {
        CommandScheduler.getInstance().reset();
    }

    @Test
    public void scheduleInitializesCommandAndLocksRequirements() {
        FakeSubsystem sub = new FakeSubsystem();
        FakeCommand cmd = new FakeCommand(sub);

        CommandScheduler.getInstance().schedule(cmd.command);

        assertEquals(1, cmd.enters.get());
        assertTrue(CommandScheduler.getInstance().isScheduled(cmd.command));
        assertTrue(CommandScheduler.getInstance().isBusy(sub));
    }

    @Test
    public void finishingReleasesRequirements() {
        FakeSubsystem sub = new FakeSubsystem();
        FakeCommand cmd = new FakeCommand(sub);

        CommandScheduler.getInstance().schedule(cmd.command);
        cmd.finished.set(true);
        CommandScheduler.getInstance().run();

        assertEquals(1, cmd.exits.get());
        assertFalse(CommandScheduler.getInstance().isScheduled(cmd.command));
        assertFalse(CommandScheduler.getInstance().isBusy(sub));
    }

    @Test
    public void conflictingScheduleCancelsAndTakesOver() {
        FakeSubsystem sub = new FakeSubsystem();
        FakeCommand a = new FakeCommand(sub);
        FakeCommand b = new FakeCommand(sub);

        CommandScheduler.getInstance().schedule(a.command);
        assertTrue(CommandScheduler.getInstance().isScheduled(a.command));

        CommandScheduler.getInstance().schedule(b.command);

        assertEquals("scheduling b should interrupt a, releasing its cleanup", 1, a.exits.get());
        assertFalse(CommandScheduler.getInstance().isScheduled(a.command));
        assertTrue(CommandScheduler.getInstance().isScheduled(b.command));
        assertTrue(CommandScheduler.getInstance().isBusy(sub));
    }

    @Test
    public void schedulingAnAlreadyActiveCommandIsANoOp() {
        FakeSubsystem sub = new FakeSubsystem();
        FakeCommand cmd = new FakeCommand(sub);

        CommandScheduler.getInstance().schedule(cmd.command);
        CommandScheduler.getInstance().schedule(cmd.command);

        assertEquals("re-scheduling an already-active command must not re-initialize it", 1, cmd.enters.get());
    }

    @Test
    public void independentSubsystemsDoNotConflict() {
        FakeSubsystem subA = new FakeSubsystem();
        FakeSubsystem subB = new FakeSubsystem();
        FakeCommand a = new FakeCommand(subA);
        FakeCommand b = new FakeCommand(subB);

        CommandScheduler.getInstance().schedule(a.command);
        CommandScheduler.getInstance().schedule(b.command);

        assertTrue(CommandScheduler.getInstance().isScheduled(a.command));
        assertTrue(CommandScheduler.getInstance().isScheduled(b.command));
        assertEquals(0, a.exits.get());
    }

    @Test
    public void runDrivesRegisteredSubsystemPeriodic() {
        FakeSubsystem sub = new FakeSubsystem();

        CommandScheduler.getInstance().run();

        assertEquals(1, sub.periodicCalls);
    }

    @Test
    public void cancelReleasesRequirementsWithoutAnotherCommand() {
        FakeSubsystem sub = new FakeSubsystem();
        FakeCommand cmd = new FakeCommand(sub);

        CommandScheduler.getInstance().schedule(cmd.command);
        CommandScheduler.getInstance().cancel(cmd.command);

        assertEquals(1, cmd.exits.get());
        assertFalse(CommandScheduler.getInstance().isScheduled(cmd.command));
        assertFalse(CommandScheduler.getInstance().isBusy(sub));
    }
}
