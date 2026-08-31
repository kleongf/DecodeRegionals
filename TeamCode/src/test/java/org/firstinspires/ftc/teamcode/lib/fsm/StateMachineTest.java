package org.firstinspires.ftc.teamcode.lib.fsm;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class StateMachineTest {

    @Test
    public void transitionAdvancesAndFallsOffEnd() {
        AtomicInteger enters = new AtomicInteger();
        AtomicInteger exits = new AtomicInteger();
        AtomicBoolean done = new AtomicBoolean(false);

        StateMachine sm = new StateMachine(
                new State()
                        .onEnter(enters::incrementAndGet)
                        .onExit(exits::incrementAndGet)
                        .transition(new Transition(done::get))
        );

        sm.start();
        assertEquals(1, enters.get());
        assertFalse(sm.isFinished());

        sm.periodic();
        assertFalse("transition condition still false", sm.isFinished());

        done.set(true);
        sm.periodic();
        assertTrue("single-state machine falls off the end once its only state finishes", sm.isFinished());
        assertEquals(1, exits.get());
    }

    @Test
    public void sequentialStatesAdvanceByIndexWhenNoNamedTransition() {
        AtomicBoolean advance = new AtomicBoolean(false);
        AtomicInteger secondStateEnters = new AtomicInteger();

        StateMachine sm = new StateMachine(
                new State().transition(new Transition(advance::get)),
                new State().onEnter(secondStateEnters::incrementAndGet)
        );

        sm.start();
        assertEquals(0, secondStateEnters.get());

        advance.set(true);
        sm.periodic();

        assertEquals(1, secondStateEnters.get());
        assertFalse("machine still has a second state to run", sm.isFinished());
    }

    @Test
    public void namedTransitionCanBranchNonSequentially() {
        AtomicBoolean jump = new AtomicBoolean(false);
        AtomicInteger parkEnters = new AtomicInteger();

        StateMachine sm = new StateMachine(
                new State("start").transition(new Transition(jump::get, "park")),
                new State("middle"),
                new State("park").onEnter(parkEnters::incrementAndGet)
        );

        sm.start();
        jump.set(true);
        sm.periodic();

        assertEquals("named transition should jump straight to \"park\", skipping \"middle\"", 1, parkEnters.get());
    }

    @Test
    public void minTimeDelaysFinishEvenIfConditionIsAlreadyTrue() {
        StateMachine sm = new StateMachine(
                new State()
                        .minTime(10_000) // effectively "never" within this fast test
                        .transition(new Transition(() -> true))
        );

        sm.start();
        sm.periodic();

        assertFalse("minTime not yet elapsed, so the state must not finish despite a true condition", sm.isFinished());
    }

    @Test
    public void maxTimeFallsBackToFinishingTheState() throws InterruptedException {
        StateMachine sm = new StateMachine(
                new State().maxTime(1) // 1ms
        );

        sm.start();
        Thread.sleep(30);
        sm.periodic();

        assertTrue("maxTime elapsed, state should finish even with no transition firing", sm.isFinished());
    }

    @Test
    public void interruptRunsOnExitOnceAndStopsTheMachine() {
        AtomicInteger exits = new AtomicInteger();
        StateMachine sm = new StateMachine(
                new State().onExit(exits::incrementAndGet)
        );

        sm.start();
        sm.interrupt();

        assertTrue(sm.isFinished());
        assertEquals(1, exits.get());

        sm.interrupt(); // idempotent — must not fire onExit a second time
        assertEquals(1, exits.get());
    }
}
