package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

public class FollowPath extends CommandBase {
    private final Follower follower;
    private final PathChain path;

    private boolean holdEnd = true;
    private double maxPower = 1;

    // Optional “trigger once at T”
    private Runnable onTTrigger = null;
    private double triggerT = 0.40;
    private boolean fired = false;

    public FollowPath(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.path = pathChain;
    }

    public FollowPath(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPath(Follower follower, PathChain pathChain, double maxPower) {
        this.follower = follower;
        this.path = pathChain;
        this.maxPower = maxPower;
    }

    public FollowPath(Follower follower, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    /** Call a function once when currentT >= t (default 0.40). */
    public FollowPath withTTrigger(double t, Runnable callback) {
        this.triggerT = t;
        this.onTTrigger = callback;
        return this;
    }

    @Override
    public void initialize() {
        fired = false;
        follower.setMaxPower(this.maxPower);
        follower.followPath(path, holdEnd);
    }

    @Override
    public void execute() {
        if (!fired && onTTrigger != null) {
            double t = follower.getCurrentTValue();
            if (t >= triggerT) {
                fired = true;
                onTTrigger.run();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return follower.atParametricEnd() && !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        follower.setMaxPower(1);
    }
}