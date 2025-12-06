package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Alliance;

public class farPaths {
    public Follower follower;
    public Pose start = new Pose(52, 8, Math.toRadians(90));
    public Pose scoreFirst = new Pose(52, 15, Math.toRadians(103));
    public Pose setFirstPick = new Pose(52, 15, Math.toRadians(180));
    public Pose firstPick = new Pose(10, 15, Math.toRadians(180));
    public Pose scoreSecond = new Pose(52, 15, Math.toRadians(103));
    public Pose setSecondPick = new Pose(52, 15, Math.toRadians(180));
    public Pose secondPick = new Pose(10, 15, Math.toRadians(180));
    public Pose thirdScore = new Pose(52, 15, Math.toRadians(103));
    public Pose park = new Pose(52, 33, Math.toRadians(90));

    public farPaths(Follower follower, Alliance alliance) {
        this.follower = follower;
        if (alliance == Alliance.RED){
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
    }

    public PathChain scoreP() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                start,
                                scoreFirst
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), scoreFirst.getHeading())
                .build();
    }
    public PathChain setOne() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scoreFirst,
                                setFirstPick
                        )
                )
                .setLinearHeadingInterpolation(scoreFirst.getHeading(), setFirstPick.getHeading())
                .build();
    }
    public PathChain pickOne() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                setFirstPick,
                                firstPick
                        )
                )
                .setLinearHeadingInterpolation(setFirstPick.getHeading(), firstPick.getHeading())
                .build();
    }
    public PathChain scoreTwo() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                firstPick,
                                scoreSecond
                        )
                )
                .setLinearHeadingInterpolation(firstPick.getHeading(), scoreSecond.getHeading())
                .build();
    }
    public PathChain setTwo() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scoreSecond,
                                setSecondPick
                        )
                )
                .setLinearHeadingInterpolation(scoreSecond.getHeading(), setSecondPick.getHeading())
                .build();
    }
    public PathChain pickTwo() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                setSecondPick,
                                secondPick
                        )
                )
                .setLinearHeadingInterpolation(setSecondPick.getHeading(), secondPick.getHeading())
                .build();
    }
    public PathChain scoreThird() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                secondPick,
                                thirdScore
                        )
                )
                .setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading())
                .build();
    }
    public PathChain parkPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                thirdScore,
                                park
                        )
                )
                .setLinearHeadingInterpolation(thirdScore.getHeading(), park.getHeading())
                .build();
    }


}
