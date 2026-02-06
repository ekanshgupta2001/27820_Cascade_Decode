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

@Autonomous(name = "Pedro Close Auto", group = "Auto")
public class closePath extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private NonVisionRobot r;
    private int pathState;
    private Alliance alliance = Alliance.BLUE;

    // Poses
    public Pose startPose = new Pose(21.913, 123, Math.toRadians(136));
    public Pose scorefirst = new Pose(21.913, 123, Math.toRadians(136));
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

    // Timing constants
    private static final double SHOOTER_SPINUP_TIME = 2.0; // Seconds to wait for shooter
    private static final double INTAKE_TIME = 0.5; // Seconds to run intake
    private static final double KICK_TIME = 0.3; // Kicker up time
    private static final double RESET_TIME = 0.2; // Kicker reset time

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
// ========== SCORE 1 (PRELOAD) ==========
            case 0: // Start driving and spinning up
                r.s.forDistance(60); // Start shooter spinup
                follower.followPath(score1);
                setPathState(1);
                break;

            case 1: // Wait for path to finish
                if (!follower.isBusy()) {
                    setPathState(2); // Path done, wait for shooter
                }
                break;

            case 2: // Wait for shooter to reach velocity
                if (r.s.isAtVelocity()) {
                    r.s.kickUp(); // Start kicking
                    setPathState(3);
                }
                break;

            case 3: // Kicker up, wait for sample to release
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown(); // Reset kicker
                    r.i.shooterinCommand();
//                    setPathState(4);
                }
                break;

            case 4: // Wait for kicker reset
                if (pathTimer.getElapsedTimeSeconds() > RESET_TIME) {
                    setPathState(5); // Ready for next pickup
                }
                break;

            case 5: // Start driving and intaking
                r.i.set(0.8); // Start intake
                follower.followPath(pick1);
                setPathState(6);
                break;

            case 6: // Wait for path to finish
                if (!follower.isBusy()) {
                    setPathState(7); // Let intake run a bit more
                }
                break;

            case 7: // Keep intaking for a moment after arriving
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    r.i.spinIdle(); // Stop intake
                    setPathState(8); // Ready to score
                }
                break;

// ========== SCORE 2 ==========
            case 8: // Start driving and spinning up
                r.s.forDistance(60);
                follower.followPath(score2);
                setPathState(9);
                break;

            case 9: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10: // Wait for shooter
                if (r.s.isAtVelocity() || pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    r.s.kickUp();
                    setPathState(11);
                }
                break;

            case 11: // Kick
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown();
                    setPathState(12);
                }
                break;

            case 12: // Reset
                if (pathTimer.getElapsedTimeSeconds() > RESET_TIME) {
                    setPathState(13);
                }
                break;

// ========== PICK 2 ==========
            case 13: // Drive and intake
                r.i.set(0.8);
//                follower.followPath(pick2);
                setPathState(14);
                break;

            case 14: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;

            case 15: // Extra intake time
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    r.i.spinIdle();
                    setPathState(16);
                }
                break;

// ========== SCORE 3 ==========
            case 16:
                r.s.forDistance(60);
//                follower.followPath(score3);
                setPathState(17);
                break;

            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;

            case 18:
                if (r.s.isAtVelocity() || pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    r.s.kickUp();
                    setPathState(19);
                }
                break;

            case 19:
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown();
                    setPathState(20);
                }
                break;

            case 20:
                if (pathTimer.getElapsedTimeSeconds() > RESET_TIME) {
                    setPathState(21);
                }
                break;

// ========== PICK 3 ==========
            case 21:
                r.i.set(0.8);
//                follower.followPath(pick3);
                setPathState(22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    setPathState(23);
                }
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    r.i.spinIdle();
                    setPathState(24);
                }
                break;

// ========== SCORE 4 ==========
            case 24:
                r.s.forDistance(60);
//                follower.followPath(score4);
                setPathState(25);
                break;

            case 25:
                if (!follower.isBusy()) {
                    setPathState(26);
                }
                break;

            case 26:
                if (r.s.isAtVelocity() || pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    r.s.kickUp();
                    setPathState(27);
                }
                break;

            case 27:
                if (pathTimer.getElapsedTimeSeconds() > KICK_TIME) {
                    r.s.kickDown();
                    setPathState(28);
                }
                break;

            case 28: // Done!
                if (pathTimer.getElapsedTimeSeconds() > 20) {
                    r.s.stopMotor();
                    setPathState(-1); // End auto
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
        r.periodic(); // Updates shooter PIDF
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Shooter Vel", r.s.getVelocity());
        telemetry.addData("Shooter Target", r.s.getTarget());
        telemetry.update();
    }
}