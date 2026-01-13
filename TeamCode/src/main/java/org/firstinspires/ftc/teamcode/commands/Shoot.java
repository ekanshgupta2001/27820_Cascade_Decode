package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.Robot;

public class Shoot extends CommandBase {
    private final NonVisionRobot r;
    private final Follower follower;

    private int st = 0;
    private final Timer t = new Timer();

    // This is the target speed this command shoots at (tune later).
    private static final double TARGET_VEL = 200;

    public Shoot(NonVisionRobot r) {
        // This command runs a full “spin up -> kick -> feed -> done” shooting cycle.
        this.r = r;
        this.follower = r.follower;
        addRequirements(r.s, r.i);
    }

    @Override
    public void initialize() {
        // This resets state/timer so each shot is consistent.
        setState(0);
    }

    @Override
    public void execute() {
        switch (st) {
            case 0:
                // This sets hood and starts spinning shooter.
                r.s.feedUp();
                r.s.setTarget(TARGET_VEL); // setTarget() now activates shooter in ShooterWait
                setState(1);
                break;

            case 1:
                // This waits until we’re basically at speed before feeding.
                if (r.s.isAtVelocity(TARGET_VEL)) {
                    setState(2);
                }
                break;

            case 2:
                // This is the timed kick/feed sequence (single cycle).
                double e = t.getElapsedTime();

                if (e > 0.00) r.s.kickUp();
                if (e > 0.30) r.s.kickDown();
                if (e > 0.60) r.i.spinIn();
                if (e > 0.90) r.i.spinIdle();

                if (e > 1.10) setState(3);
                break;

            case 3:
                // This ends once we’re done and follower isn’t busy (optional).
                if (!follower.isBusy()) {
                    setState(-1);
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // This ends when state machine says we’re done.
        return st == -1;
    }

    @Override
    public void end(boolean interrupted) {
        // This leaves the robot in a safe “not shooting” state.
        r.i.spinIdle();
        r.s.kickDown();
        r.s.stopMotor();
    }

    private void setState(int state) {
        // This changes states cleanly and restarts the timer for that state.
        st = state;
        t.resetTimer();
    }
}