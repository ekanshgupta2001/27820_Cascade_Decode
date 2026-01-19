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
    public static double SPINUP_TIMEOUT = 3.5;    // seconds (you said ~3s)
    public static double KICK_UP_TIME = 0.25;
    public static double KICK_DOWN_TIME = 0.25;
    public static double FEED_TIME = 0.40;

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

    public Shoot(NonVisionRobot r) {
        this.r = r;
        addRequirements(r.s, r.i);
    }

    @Override
    public void initialize() {
        // Make sure shooter target matches current pose at the moment shooting starts
        Pose robotPose = r.follower.getPose();
        Pose targetPose = r.getShootTarget();

        if (robotPose != null && targetPose != null) {
            double dx = Math.abs(targetPose.getX() - robotPose.getX());
            double dy = Math.abs(targetPose.getY() - robotPose.getY());
            r.s.forDistance(dx, dy);
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
                // Keep holding the shooter target while waiting
                Pose robotPose = r.follower.getPose();
                Pose targetPose = r.getShootTarget();
                if (robotPose != null && targetPose != null) {
                    double dx = Math.abs(targetPose.getX() - robotPose.getX());
                    double dy = Math.abs(targetPose.getY() - robotPose.getY());
                    r.s.forDistance(dx, dy);
                }

                boolean atSpeed = r.s.isAtVelocity(r.s.getTarget());
                boolean timedOut = timer.getElapsedTime() >= SPINUP_TIMEOUT;

                if (atSpeed || timedOut) {
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
        r.s.stopMotor();
    }
}