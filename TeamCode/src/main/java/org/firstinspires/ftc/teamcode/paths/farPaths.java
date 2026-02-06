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
import org.firstinspires.ftc.teamcode.paths.autoShooterSequence;

@Autonomous(name = "Pedro Far Auto (Fixed)", group = "Auto")
public class farPaths extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private NonVisionRobot r;
    private int pathState;
    private Alliance alliance = Alliance.BLUE;

    private autoShooterSequence shooterSeq;  // ADD THIS

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

    // Timing
    private static final double INTAKE_TIME = 0.65;
    private static final double POST_PATH_SETTLE = 0.10;

    // Custom mirror for Red
    private Pose mirrorForRed(Pose bluePose) {
        return new Pose(
                bluePose.getX(),
                -bluePose.getY(),
                2 * Math.PI - bluePose.getHeading()
        );
    }

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
        // Update shooter sequence if running
        if (shooterSeq.isRunning()) {
            shooterSeq.update();
        }

        switch (pathState) {
            // ========== SCORE 1 (PRELOAD) ==========
            case 0: // Drive to score position
                follower.followPath(score1);
                setPathState(1);
                break;

            case 1: // Wait for path to finish
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2: // Settle and start 3-shot sequence
                if (pathTimer.getElapsedTimeSeconds() >= POST_PATH_SETTLE) {
                    shooterSeq.start(60);  // Start 3-shot sequence
                    setPathState(3);
                }
                break;

            case 3: // Wait for all 3 shots to complete
                if (shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    setPathState(4);
                }
                break;

            // ========== SETUP PICK 1 ==========
            case 4: // Drive to setup position
                follower.followPath(s1);
                setPathState(5);
                break;

            case 5: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            // ========== PICK 1 ==========
            case 6: // Start intake and drive to sample
                r.i.set(0.8);
                follower.followPath(p1);
                setPathState(7);
                break;

            case 7: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8: // Extra intake time
                if (pathTimer.getElapsedTimeSeconds() >= INTAKE_TIME) {
                    r.i.spinIdle();
                    setPathState(9);
                }
                break;

            // ========== SCORE 2 ==========
            case 9: // Drive to score
                follower.followPath(score2);
                setPathState(10);
                break;

            case 10: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11: // Settle and shoot
                if (pathTimer.getElapsedTimeSeconds() >= POST_PATH_SETTLE) {
                    shooterSeq.start(60);
                    setPathState(12);
                }
                break;

            case 12: // Wait for shots
                if (shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    setPathState(13);
                }
                break;

            // ========== SETUP PICK 2 ==========
            case 13: // Drive to setup
                follower.followPath(s2);
                setPathState(14);
                break;

            case 14: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;

            // ========== PICK 2 ==========
            case 15: // Start intake and drive
                r.i.set(0.8);
                follower.followPath(p2);
                setPathState(16);
                break;

            case 16: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;

            case 17: // Extra intake time
                if (pathTimer.getElapsedTimeSeconds() >= INTAKE_TIME) {
                    r.i.spinIdle();
                    setPathState(18);
                }
                break;

            // ========== SCORE 3 ==========
            case 18: // Drive to score
                follower.followPath(score3);
                setPathState(19);
                break;

            case 19: // Wait for path
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;

            case 20: // Settle and shoot
                if (pathTimer.getElapsedTimeSeconds() >= POST_PATH_SETTLE) {
                    shooterSeq.start(60);
                    setPathState(21);
                }
                break;

            case 21: // Wait for shots
                if (shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    setPathState(22);
                }
                break;

            // ========== PARK ==========
            case 22: // Drive to park
                r.s.stopMotor();
                follower.followPath(parkP);
                setPathState(23);
                break;

            case 23: // Wait for park to finish
                if (!follower.isBusy()) {
                    setPathState(-1);  // Done!
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
        shooterSeq = new autoShooterSequence(r);  // ADD THIS

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
            start = mirrorForRed(start);
            scoreFirst = mirrorForRed(scoreFirst);
            setFirstPick = mirrorForRed(setFirstPick);
            firstPick = mirrorForRed(firstPick);
            scoreSecond = mirrorForRed(scoreSecond);
            setSecondPick = mirrorForRed(setSecondPick);
            secondPick = mirrorForRed(secondPick);
            thirdScore = mirrorForRed(thirdScore);
            park = mirrorForRed(park);
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

        Pose p = follower.getPose();
        telemetry.addData("=== STATE ===", "");
        telemetry.addData("Path State", pathState);

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("Pose", p == null ? "null" :
                String.format("(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetry.addData("Path Busy?", follower.isBusy() ? "YES" : "NO");

        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Vel / Target", String.format("%.0f / %.0f", r.s.getVelocity(), r.s.getTarget()));
        telemetry.addData("At Speed?", r.s.isAtVelocity() ? "YES" : "NO");

        telemetry.update();
    }
}