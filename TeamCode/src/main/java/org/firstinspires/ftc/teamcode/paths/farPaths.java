package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Alliance;

public class farPaths {
    public Follower follower;

    public Pose start = new Pose(56, 8, Math.toRadians(90));
    public Pose scoreFirst = new Pose(56, 15, Math.toRadians(108));

    public Pose setFirstPick = new Pose(40, 36, Math.toRadians(180));
    public Pose firstPick = new Pose(6, 36, Math.toRadians(180));

    public Pose scoreSecond = new Pose(56, 15, Math.toRadians(108));

    public Pose setSecondPick = new Pose(9, 13, Math.toRadians(180));
    public Pose secondPick = new Pose(9, 6, Math.toRadians(180));

    public Pose thirdScore = new Pose(56, 15, Math.toRadians(108));
    public Pose park = new Pose(38, 12, Math.toRadians(90));

    private int index;

    private static final int PATH_COUNT = 8; // cases 0..7

    public farPaths(Follower follower, Alliance alliance) {
        this.follower = follower;

        if (alliance == Alliance.RED) {
            start = start.mirror();
            scoreFirst = scoreFirst.mirror();
            setFirstPick = setFirstPick.mirror();
            firstPick = firstPick.mirror();
            scoreSecond = scoreSecond.mirror();
            setSecondPick = setSecondPick.mirror();
            secondPick = secondPick.mirror();
            thirdScore = thirdScore.mirror();
            park = park.mirror();
        }

        reset();
    }

    public PathChain scoreP() {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, scoreFirst))
                .setLinearHeadingInterpolation(start.getHeading(), scoreFirst.getHeading())
                .build();
    }

    public PathChain setOne() {
        return follower.pathBuilder()
                .addPath(new BezierLine(scoreFirst, setFirstPick))
                .setLinearHeadingInterpolation(scoreFirst.getHeading(), setFirstPick.getHeading())
                .build();
    }

    public PathChain pickOne() {
        return follower.pathBuilder()
                .addPath(new BezierLine(setFirstPick, firstPick))
                .setLinearHeadingInterpolation(setFirstPick.getHeading(), firstPick.getHeading())
                .build();
    }

    public PathChain scoreTwo() {
        return follower.pathBuilder()
                .addPath(new BezierLine(firstPick, scoreSecond))
                .setLinearHeadingInterpolation(firstPick.getHeading(), scoreSecond.getHeading())
                .build();
    }

    public PathChain setTwo() {
        return follower.pathBuilder()
                .addPath(new BezierLine(scoreSecond, setSecondPick))
                .setLinearHeadingInterpolation(scoreSecond.getHeading(), setSecondPick.getHeading())
                .build();
    }

    public PathChain pickTwo() {
        return follower.pathBuilder()
                .addPath(new BezierLine(setSecondPick, secondPick))
                .setLinearHeadingInterpolation(setSecondPick.getHeading(), secondPick.getHeading())
                .build();
    }

    public PathChain scoreThird() {
        return follower.pathBuilder()
                .addPath(new BezierLine(secondPick, thirdScore))
                .setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading())
                .build();
    }

    public PathChain parkPath() {
        return follower.pathBuilder()
                .addPath(new BezierLine(thirdScore, park))
                .setLinearHeadingInterpolation(thirdScore.getHeading(), park.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            case 1: return setOne();
            case 2: return pickOne();
            case 3: return scoreTwo();
            case 4: return setTwo();
            case 5: return pickTwo();
            case 6: return scoreThird();
            case 7: return parkPath();
            default: return null;
        }
    }

    public boolean hasNext() {
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}