package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Alliance;

public class closePath {
    public Follower follower;
    public Pose start = new Pose(21.913, 123, Math.toRadians(136));
    public Pose scorefirst = new Pose(60, 84, Math.toRadians(136));
    public Pose setFirstPick = new Pose(60, 84, Math.toRadians(180));
    public Pose firstPick = new Pose(16.5, 84, Math.toRadians(180));
    public Pose scoreSecond = new Pose(60, 84, Math.toRadians(136));
    public Pose setSecondPick = new Pose(45, 60, Math.toRadians(180));
    public Pose secondPick = new Pose(10, 60, Math.toRadians(180));
    public Pose thirdScore = new Pose(45, 60, Math.toRadians(145));
    public Pose setThirdPick = new Pose(60, 84, Math.toRadians(136));
    public Pose thirdPick = new Pose(10, 36, Math.toRadians(180));
    public Pose fourthScore = new Pose(60, 84, Math.toRadians(136));
    private int index;
    public closePath(Follower follower, Alliance alliance) {
        this.follower = follower;
        if (alliance == Alliance.RED){
            start = start.mirror();
            scorefirst = scorefirst.mirror();
            setFirstPick = setFirstPick.mirror();
            firstPick = firstPick.mirror();
            scoreSecond = scoreSecond.mirror();
            setSecondPick = setSecondPick.mirror();
            secondPick = secondPick.mirror();
            thirdScore = thirdScore.mirror();
            setThirdPick = setThirdPick.mirror();
            thirdPick = thirdPick.mirror();
            fourthScore = fourthScore.mirror();
        }
    }
    public PathChain scoreP() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                start,
                                scorefirst
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), scorefirst.getHeading())
                .build();
    }
    public PathChain setOne() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorefirst,
                                setFirstPick
                        )
                )
                .setLinearHeadingInterpolation(scorefirst.getHeading(), setFirstPick.getHeading())
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
    public PathChain setThird() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                thirdScore,
                                setThirdPick
                        )
                )
                .setLinearHeadingInterpolation(thirdScore.getHeading(), setThirdPick.getHeading())
                .build();
    }
    public PathChain pickThree() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                setThirdPick,
                                thirdPick
                        )
                )
                .setLinearHeadingInterpolation(setThirdPick.getHeading(), thirdPick.getHeading())
                .build();
    }
    public PathChain scoreFourth() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                thirdPick,
                                fourthScore
                        )
                )
                .setLinearHeadingInterpolation(thirdPick.getHeading(), fourthScore.getHeading())
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
            case 7: return setThird();
            case 8: return pickThree();
            case 9: return scoreFourth();
            default: return null;
        }
    }
    public boolean hasNext() {
        int PATH_COUNT = 9;
        return index < PATH_COUNT;
    }
    public void reset() {
        index = 0;
    }

}
