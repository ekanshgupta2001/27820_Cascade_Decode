package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

public class FollowPath extends CommandBase {
    private final Follower follower;
    private final PathChain path;

    private boolean holdEnd = true;
    private double maxPower = 1;

    public FollowPath(Follower follower, PathChain pathChain) {
        // This stores follower + path so we can start it cleanly in initialize().
        this.follower = follower;
        this.path = pathChain;
    }

    public FollowPath(Follower follower, PathChain pathChain, double maxPower) {
        // This lets us slow down path following for accuracy.
        this.follower = follower;
        this.path = pathChain;
        this.maxPower = maxPower;
    }

    public FollowPath(Follower follower, PathChain pathChain, boolean holdEnd) {
        // This decides if the robot “holds” its final pose once it’s done.
        this.follower = follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPath(Follower follower, PathChain pathChain, boolean holdEnd, double maxPower) {
        // This constructor gives full control over hold and speed.
        this.follower = follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowPath setHoldEnd(boolean holdEnd) {
        // This is a chainable setter for “hold at the end.”
        this.holdEnd = holdEnd;
        return this;
    }

    public FollowPath setMaxPower(double power) {
        // This is a chainable setter for max follower power (0..1).
        this.maxPower = power;
        return this;
    }

    @Override
    public void initialize() {
        // This starts path following at the requested speed.
        follower.setMaxPower(this.maxPower);
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        // This ends when Pedro says we’re at the end of the path.
        return follower.atParametricEnd();
    }

    @Override
    public void end(boolean interrupted) {
        // This restores max power so the next path/teleop isn’t accidentally capped.
        follower.setMaxPower(1);
    }
}