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
    public double dist_x;
    public double dist_y;

    // Inside Shoot.java
    public Shoot(NonVisionRobot r, double dx, double dy) {
        this.r = r;
        this.follower = r.follower;
        this.dist_x = dx;
        this.dist_y = dy;
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
                r.s.forDistance(dist_x, dist_y); // setTarget() now activates shooter in ShooterWait
                setState(1);
                break;

            case 1:
                // This waits until we’re basically at speed before feeding.
                if (r.s.isAtVelocity(r.s.getTarget())) {
                    setState(2);
                }
                break;

            case 2:
                double e = t.getElapsedTime();

                // Shot 1
                if (e >= 0.00 && e < 0.30) r.s.kickUp();
                else if (e >= 0.30 && e < 0.60) r.s.kickDown();

                // Intake/Transfer for Shot 2
                if (e >= 0.60 && e < 0.90) r.i.shooterinCommand();

                    // Shot 2
                else if (e >= 0.90 && e < 1.20) r.s.kickUp();
                else if (e >= 1.20 && e < 1.50) r.s.kickDown();

                // Intake/Transfer for Shot 3
                if (e >= 1.50 && e < 1.80) r.i.shooterinCommand();

                    // Shot 3
                else if (e >= 1.80 && e < 2.10) r.s.kickUp();
                else if (e >= 2.10 && e < 2.40) r.s.kickDown();

                // Cleanup and Exit
                if (e > 2.50) {
                    r.i.spinIdle();
                    setState(3);
                }
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