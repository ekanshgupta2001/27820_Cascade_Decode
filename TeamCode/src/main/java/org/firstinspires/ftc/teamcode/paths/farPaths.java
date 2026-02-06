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

@Autonomous(name = "Pedro Far Auto", group = "Auto")
public class farPaths extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private NonVisionRobot r;
    private int pathState;
    private Alliance alliance = Alliance.BLUE;

    // Poses
    public Pose start = new Pose(56, 8, Math.toRadians(90));
    public Pose scoreFirst = new Pose(56, 15, Math.toRadians(102));
    public Pose setFirstPick = new Pose(40, 36, Math.toRadians(180));
    public Pose firstPick = new Pose(6, 36, Math.toRadians(180));
    public Pose scoreSecond = new Pose(56, 15, Math.toRadians(108));
    public Pose setSecondPick = new Pose(9, 13, Math.toRadians(180));
    public Pose secondPick = new Pose(9, 6, Math.toRadians(180));
    public Pose thirdScore = new Pose(56, 15, Math.toRadians(108));
    public Pose park = new Pose(38, 12, Math.toRadians(90));

    // PathChains
    private PathChain score1, s1, p1, score2, s2, p2, score3, parkP;

    // Timing constants
    private static final double SHOOTER_SPINUP_TIME = 2.0; // Seconds to wait for shooter
    private static final double INTAKE_TIME = 0.5; // Seconds to run intake
    private static final double KICK_TIME = 0.3; // Kicker up time
    private static final double RESET_TIME = 0.2; // Kicker reset time

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(start, scoreFirst))
                .setLinearHeadingInterpolation(start.getHeading(), scoreFirst.getHeading())
                .build();

        s1 = follower.pathBuilder()
                .addPath(new BezierLine(scoreFirst, setFirstPick))
                .setLinearHeadingInterpolation(scoreFirst.getHeading(), setFirstPick.getHeading())
                .build();

        p1 = follower.pathBuilder()
                .addPath(new BezierLine(setFirstPick, firstPick))
                .setLinearHeadingInterpolation(setFirstPick.getHeading(), firstPick.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(firstPick, scoreSecond))
                .setLinearHeadingInterpolation(firstPick.getHeading(), scoreSecond.getHeading())
                .build();

        s2 = follower.pathBuilder()
                .addPath(new BezierLine(scoreSecond, setSecondPick))
                .setLinearHeadingInterpolation(scoreSecond.getHeading(), setSecondPick.getHeading())
                .build();

        p2 = follower.pathBuilder()
                .addPath(new BezierLine(setSecondPick, secondPick))
                .setLinearHeadingInterpolation(setSecondPick.getHeading(), secondPick.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(secondPick, thirdScore))
                .setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading())
                .build();

        parkP = follower.pathBuilder()
                .addPath(new BezierLine(thirdScore, park))
                .setLinearHeadingInterpolation(thirdScore.getHeading(), park.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
// ========== SCORE 1 (PRELOAD) ==========
            case 0: // Start driving and spinning up
                r.s.forDistance(100); // Start shooter spinup
                follower.followPath(score1);
                setPathState(1);
                break;

            case 1: // Wait for path to finish
                if (!follower.isBusy() && r.s.isAtVelocity()) {
                    r.s.kickUp();
                    setPathState(3);
                }
                break;
            case 3: // Kicker up
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown();
                    setPathState(4);
                }
                break;

            case 4: // Reset kicker
                if (pathTimer.getElapsedTimeSeconds() > RESET_TIME) {
                    setPathState(5);
                }
                break;

// ========== SETUP PICK 1 ==========
            case 5: // Drive to setup position
                follower.followPath(s1);
                setPathState(6);
                break;

            case 6: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

// ========== PICK 1 ==========
            case 7: // Start intake and drive to sample
                r.i.set(0.8);
                follower.followPath(p1);
                setPathState(8);
                break;

            case 8: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;

            case 9: // Extra intake time
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    r.i.spinIdle();
                    setPathState(10);
                }
                break;

// ========== SCORE 2 ==========
            case 10: // Drive and spinup
                r.s.forDistance(60);
                follower.followPath(score2);
                setPathState(11);
                break;

            case 11: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12: // Wait for shooter
                if (r.s.isAtVelocity() || pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    r.s.kickUp();
                    setPathState(13);
                }
                break;

            case 13: // Kick
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown();
                    setPathState(14);
                }
                break;

            case 14: // Reset
                if (pathTimer.getElapsedTimeSeconds() > RESET_TIME) {
                    setPathState(15);
                }
                break;

// ========== SETUP PICK 2 ==========
            case 15: // Drive to setup
                follower.followPath(s2);
                setPathState(16);
                break;

            case 16: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;

// ========== PICK 2 ==========
            case 17: // Start intake and drive
                r.i.set(0.8);
                follower.followPath(p2);
                setPathState(18);
                break;

            case 18: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(19);
                }
                break;

            case 19: // Extra intake time
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    r.i.spinIdle();
                    setPathState(20);
                }
                break;

// ========== SCORE 3 ==========
            case 20: // Drive and spinup
                r.s.forDistance(60);
                follower.followPath(score3);
                setPathState(21);
                break;

            case 21: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;

            case 22: // Wait for shooter
                if (r.s.isAtVelocity() || pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    r.s.kickUp();
                    setPathState(23);
                }
                break;

            case 23: // Kick
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown();
                    setPathState(24);
                }
                break;

            case 24: // Reset
                if (pathTimer.getElapsedTimeSeconds() > RESET_TIME) {
                    setPathState(25);
                }
                break;

// ========== PARK ==========
            case 25: // Drive to park
                r.s.stopMotor(); // Stop shooter
                follower.followPath(parkP);
                setPathState(26);
                break;

            case 26: // Wait for park to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // Done!
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
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        follower = r.follower;

// Set shooter to idle speed
        r.s.setTarget(250);
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
        telemetry.addData("Shooter Vel", r.s.getVelocity());
        telemetry.addData("Shooter Target", r.s.getTarget());
        telemetry.addData("Shooter At Speed?", r.s.isAtVelocity() ? "YES" : "NO");
        telemetry.update();
    }
}