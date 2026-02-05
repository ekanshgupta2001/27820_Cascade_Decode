package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Pedro Far Auto", group = "Auto")
public class farPaths extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private NonVisionRobot r;
    private int pathState;
    private Alliance alliance = Alliance.BLUE;

    // Poses
    public Pose start = new Pose(56, 8, Math.toRadians(90));
    public Pose scoreFirst = new Pose(56, 15, Math.toRadians(108));
    public Pose setFirstPick = new Pose(40, 36, Math.toRadians(180));
    public Pose firstPick = new Pose(6, 36, Math.toRadians(180));
    public Pose scoreSecond = new Pose(56, 15, Math.toRadians(108));
    public Pose setSecondPick = new Pose(9, 13, Math.toRadians(180));
    public Pose secondPick = new Pose(9, 6, Math.toRadians(180));
    public Pose thirdScore = new Pose(56, 15, Math.toRadians(108));
    public Pose park = new Pose(38, 12, Math.toRadians(90));

    // PathChains
    private PathChain score1, s1, p1, score2, s2, p2, score3, parkP;

    public void buildPaths() {
        score1 = follower.pathBuilder().addPath(new BezierLine(start, scoreFirst)).setLinearHeadingInterpolation(start.getHeading(), scoreFirst.getHeading()).build();
        s1 = follower.pathBuilder().addPath(new BezierLine(scoreFirst, setFirstPick)).setLinearHeadingInterpolation(scoreFirst.getHeading(), setFirstPick.getHeading()).build();
        p1 = follower.pathBuilder().addPath(new BezierLine(setFirstPick, firstPick)).setLinearHeadingInterpolation(setFirstPick.getHeading(), firstPick.getHeading()).build();
        score2 = follower.pathBuilder().addPath(new BezierLine(firstPick, scoreSecond)).setLinearHeadingInterpolation(firstPick.getHeading(), scoreSecond.getHeading()).build();
        s2 = follower.pathBuilder().addPath(new BezierLine(scoreSecond, setSecondPick)).setLinearHeadingInterpolation(scoreSecond.getHeading(), setSecondPick.getHeading()).build();
        p2 = follower.pathBuilder().addPath(new BezierLine(setSecondPick, secondPick)).setLinearHeadingInterpolation(setSecondPick.getHeading(), secondPick.getHeading()).build();
        score3 = follower.pathBuilder().addPath(new BezierLine(secondPick, thirdScore)).setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading()).build();
        parkP = follower.pathBuilder().addPath(new BezierLine(thirdScore, park)).setLinearHeadingInterpolation(thirdScore.getHeading(), park.getHeading()).build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Score 1
                r.s.forDistance(60);
                follower.followPath(score1);
                setPathState(1);
                break;
            case 1: // Shoot 1
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(2);
                }
                break;
            case 2: // Setup Pick 1
                if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(s1);
                    setPathState(3);
                }
                break;
            case 3: // Intake Pick 1
                if(!follower.isBusy()) {
                    new IntakeIn(r.i);
                    follower.followPath(p1);
                    setPathState(4);
                }
                break;
            case 4: // Score 2
                if(!follower.isBusy()) {
                    r.s.forDistance(60);
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;
            case 5: // Shoot 2
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(6);
                }
                break;
            case 6: // Setup Pick 2
                if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(s2);
                    setPathState(7);
                }
                break;
            case 7: // Intake Pick 2
                if(!follower.isBusy()) {
                    new IntakeIn(r.i);
                    follower.followPath(p2);
                    setPathState(8);
                }
                break;
            case 8: // Score 3
                if(!follower.isBusy()) {
                    r.s.forDistance(60);
                    follower.followPath(score3);
                    setPathState(9);
                }
                break;
            case 9: // Shoot 3
                if(!follower.isBusy()) {
                    new Shoot(r);
                    setPathState(10);
                }
                break;
            case 10: // Park
                if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(parkP);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) setPathState(-1);
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
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        follower = r.follower;
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;
        telemetry.addData("Alliance Selected", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
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
        buildPaths();
        follower.setStartingPose(start);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        r.periodic();
        autonomousPathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("Pose", follower.getPose().toString());
        telemetry.update();
    }
}
