package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.NonVisionRobot;

@Config
public class Shoot extends CommandBase {

    private final NonVisionRobot r;
    private final Timer timer = new Timer();

    // Tune these in dashboard
    public static double MIN_SPINUP_TIME = 0.5;     // Don't wait forever
    public static double SPINUP_TIMEOUT = 4.0;      // Accept 4 second max
    public static double KICK_UP_TIME = 0.25;
    public static double KICK_DOWN_TIME = 0.25;
    public static double FEED_TIME = 0.5;

    private enum State {
        WAIT_FOR_SPEED,
        SHOT1_UP,
        SHOT1_DOWN_FEED,
        SHOT2_UP,
        SHOT2_DOWN_FEED,
        SHOT3_UP,
        SHOT3_DOWN,
        DONE
    }

    private State st = State.WAIT_FOR_SPEED;

    // ADDED: Store the calculated distance once at the start
    private double targetDistance = 0;

    public Shoot(NonVisionRobot r) {
        this.r = r;
        addRequirements(r.s, r.i);
    }

    @Override
    public void initialize() {
        // CHANGED: Calculate target distance ONCE and store it
        Pose robotPose = r.follower.getPose();
        Pose targetPose = r.getShootTarget();

        if (robotPose != null && targetPose != null) {
            // FIXED: Only use Y distance (vertical distance to basket)
            targetDistance = Math.abs(targetPose.getY() - robotPose.getY());

            // Set shooter velocity based on this fixed distance
            r.s.forDistance(targetDistance);  // FIXED: First param unused, see FIXME
        } else {
            // Fallback if poses are null
            targetDistance = 60;  // Default medium distance
            r.s.forDistance(targetDistance);
        }

        r.s.kickDown();
        r.i.spinIdle();

        st = State.WAIT_FOR_SPEED;
        timer.resetTimer();
    }

    @Override
    public void execute() {
        switch (st) {
            case WAIT_FOR_SPEED: {
                // REMOVED: No longer recalculating distance here!
                // The shooter target is already set in initialize()

                double elapsed = timer.getElapsedTime();
                boolean atSpeed = r.s.isAtVelocity(r.s.getTarget());
                boolean minTimeReached = elapsed >= MIN_SPINUP_TIME;
                boolean timedOut = elapsed >= SPINUP_TIMEOUT;

                // CHANGED: Must meet BOTH conditions OR timeout
                if ((atSpeed && minTimeReached) || timedOut) {
                    st = State.SHOT1_UP;
                    timer.resetTimer();
                }
                break;
            }

            case SHOT1_UP:
                r.s.kickUp();
                if (timer.getElapsedTime() >= KICK_UP_TIME) {
                    st = State.SHOT1_DOWN_FEED;
                    timer.resetTimer();
                }
                break;

            case SHOT1_DOWN_FEED:
                r.s.kickDown();
                r.i.intakeShooter();
                if (timer.getElapsedTime() >= (KICK_DOWN_TIME + FEED_TIME)) {
                    st = State.SHOT2_UP;
                    timer.resetTimer();
                }
                break;

            case SHOT2_UP:
                r.s.kickUp();
                if (timer.getElapsedTime() >= KICK_UP_TIME) {
                    st = State.SHOT2_DOWN_FEED;
                    timer.resetTimer();
                }
                break;

            case SHOT2_DOWN_FEED:
                r.s.kickDown();
                r.i.intakeShooter();
                if (timer.getElapsedTime() >= (KICK_DOWN_TIME + FEED_TIME)) {
                    st = State.SHOT3_UP;
                    timer.resetTimer();
                }
                break;

            case SHOT3_UP:
                r.s.kickUp();
                if (timer.getElapsedTime() >= KICK_UP_TIME) {
                    st = State.SHOT3_DOWN;
                    timer.resetTimer();
                }
                break;

            case SHOT3_DOWN:
                r.s.kickDown();
                if (timer.getElapsedTime() >= KICK_DOWN_TIME) {
                    st = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return st == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        r.i.spinIdle();
        r.s.kickDown();
        r.s.setTarget(290);
    }
}