package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;

@Autonomous(name = "Pedro Close Auto", group = "Auto")
public class closePath extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private NonVisionRobot r;
    private int pathState;
    private Alliance alliance = Alliance.BLUE;

    // Poses
    public Pose startPose = new Pose(21.913, 123, Math.toRadians(136));
    public Pose scorefirst = new Pose(45, 100, Math.toRadians(136));
    public Pose setFirstPick = new Pose(45, 84, Math.toRadians(180));
    public Pose firstPick = new Pose(16.5, 84, Math.toRadians(180));
    public Pose scoreSecond = new Pose(45, 100, Math.toRadians(136));
    public Pose setSecondPick = new Pose(45, 60, Math.toRadians(180));
    public Pose secondPick = new Pose(10, 60, Math.toRadians(180));
    public Pose setThirdScore = new Pose(45, 60, Math.toRadians(136));
    public Pose thirdScore = new Pose(45, 100, Math.toRadians(136));
    public Pose setThirdPick = new Pose(45, 36, Math.toRadians(180));
    public Pose thirdPick = new Pose(10, 36, Math.toRadians(180));
    public Pose fourthScore = new Pose(45, 100, Math.toRadians(136));

    // PathChains
    private PathChain score1, pick1, score2, pick2, score3, pick3, score4;

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorefirst))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorefirst.getHeading())
                .build();

        pick1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorefirst, setFirstPick, firstPick))
                .setLinearHeadingInterpolation(scorefirst.getHeading(), firstPick.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(firstPick, scoreSecond))
                .setLinearHeadingInterpolation(firstPick.getHeading(), scoreSecond.getHeading())
                .build();

        pick2 = follower.pathBuilder()
                .addPath(new BezierCurve(scoreSecond, setSecondPick, secondPick))
                .setLinearHeadingInterpolation(scoreSecond.getHeading(), secondPick.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(secondPick, setThirdScore, thirdScore))
                .setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading())
                .build();

        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(thirdScore, setThirdPick, thirdPick))
                .setLinearHeadingInterpolation(thirdScore.getHeading(), thirdPick.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(thirdPick, fourthScore))
                .setLinearHeadingInterpolation(thirdPick.getHeading(), fourthScore.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Score 1
                r.s.forDistance(60);
                follower.followPath(score1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(2);
                }
                break;
            case 2: // Pick 1
                if(pathTimer.getElapsedTimeSeconds() > 4.0) {
                    new IntakeIn(r.i);
                    follower.followPath(pick1);
                    setPathState(3);
                }
                break;
            case 3: // Score 2
                if(!follower.isBusy()) {
                    r.s.forDistance(60);
                    follower.followPath(score2);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(5);
                }
                break;
            case 5: // Pick 2
                if(pathTimer.getElapsedTimeSeconds() > 4.0) {
                    new IntakeIn(r.i);
                    follower.followPath(pick2);
                    setPathState(6);
                }
                break;
            case 6: // Score 3
                if(!follower.isBusy()) {
                    r.s.forDistance(60);
                    follower.followPath(score3);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(8);
                }
                break;
            case 8: // Pick 3
                if(pathTimer.getElapsedTimeSeconds() > 4.0) {
                    new IntakeIn(r.i);
                    follower.followPath(pick3);
                    setPathState(9);
                }
                break;
            case 9: // Score 4
                if(!follower.isBusy()) {
                    r.s.forDistance(60);
                    follower.followPath(score4);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE); // Init with default
        follower = r.follower;
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        if (alliance == Alliance.RED) {
            startPose = startPose.mirror();
            scorefirst = scorefirst.mirror();
            setFirstPick = setFirstPick.mirror();
            firstPick = firstPick.mirror();
            scoreSecond = scoreSecond.mirror();
            setSecondPick = setSecondPick.mirror();
            secondPick = secondPick.mirror();
            setThirdScore = setThirdScore.mirror();
            thirdScore = thirdScore.mirror();
            setThirdPick = setThirdPick.mirror();
            thirdPick = thirdPick.mirror();
            fourthScore = fourthScore.mirror();
        }
        buildPaths();
        follower.setStartingPose(startPose);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        r.periodic();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}
