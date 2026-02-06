package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.NonVisionRobot;

public class autoShooterSequence {
    public static double MIN_SPINUP_TIME = 1.0;
    public static double SPINUP_TIMEOUT = 2.5;
    public static double KICK_UP_TIME = 0.3;
    public static double KICK_DOWN_TIME = 0.2;
    public static double FEED_TIME = 0.8;

    private enum State {
        IDLE, WAIT_FOR_SPEED,
        SHOT1_UP, SHOT1_DOWN_FEED,
        SHOT2_UP, SHOT2_DOWN_FEED,
        SHOT3_UP, SHOT3_DOWN,
        DONE
    }

    NonVisionRobot r;
    private final Timer timer = new Timer();
    private State state = State.IDLE;

    public autoShooterSequence(NonVisionRobot robot) {
        this.r = robot;
    }

    public void start(double distance) {
        r.s.forDistance(distance);
        r.s.kickDown();
        r.i.spinIdle();
        state = State.WAIT_FOR_SPEED;
        timer.resetTimer();
    }

    public void update() {
        if (state == State.IDLE || state == State.DONE) return;

        double elapsed = timer.getElapsedTimeSeconds();
        boolean atSpeed = r.s.isAtVelocity();

        switch (state) {
            case WAIT_FOR_SPEED:
                if ((atSpeed && elapsed >= MIN_SPINUP_TIME) || elapsed >= SPINUP_TIMEOUT) {
                    state = State.SHOT1_UP;
                    timer.resetTimer();
                }
                break;

            case SHOT1_UP:
                r.s.kickUp();
                if (elapsed >= KICK_UP_TIME) {
                    state = State.SHOT1_DOWN_FEED;
                    timer.resetTimer();
                }
                break;

            case SHOT1_DOWN_FEED:
                r.s.kickDown();
                if (elapsed < FEED_TIME) r.i.set(0.7); else r.i.spinIdle();

                // FIX: Check if kicker is down AND flywheel speed has recovered
                if (elapsed >= (KICK_DOWN_TIME + FEED_TIME) && atSpeed) {
                    state = State.SHOT2_UP;
                    timer.resetTimer();
                }
                break;

            case SHOT2_UP:
                r.s.kickUp();
                if (elapsed >= KICK_UP_TIME) {
                    state = State.SHOT2_DOWN_FEED;
                    timer.resetTimer();
                }
                break;

            case SHOT2_DOWN_FEED:
                r.s.kickDown();
                if (elapsed < FEED_TIME) r.i.set(0.7); else r.i.spinIdle();

                // FIX: Velocity recovery check
                if (elapsed >= (KICK_DOWN_TIME + FEED_TIME) && atSpeed) {
                    state = State.SHOT3_UP;
                    timer.resetTimer();
                }
                break;

            case SHOT3_UP:
                r.s.kickUp();
                if (elapsed >= KICK_UP_TIME) {
                    state = State.SHOT3_DOWN;
                    timer.resetTimer();
                }
                break;

            case SHOT3_DOWN:
                r.s.kickDown();
                if (elapsed >= KICK_DOWN_TIME) state = State.DONE;
                break;
        }
    }

    public boolean isRunning() { return state != State.IDLE && state != State.DONE; }
    public boolean isDone() { return state == State.DONE; }
    public void resetToIdle() { state = State.IDLE; }
}