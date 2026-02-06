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

@Autonomous(name = "Pedro Close Auto Fixed", group = "Auto")
public class closePath extends OpMode {

    private Follower follower;
    private final Timer pathTimer = new Timer();
    private NonVisionRobot r;
    private int pathState;
    private Alliance alliance = Alliance.BLUE;
    private autoShooterSequence shooterSeq;

    // BASE BLUE POSES
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

    private PathChain s1, p1, s2, p2, s3, p3, s4;

    // FIX: Correct Mirroring for 0-144 field
    private Pose mirror(Pose blue) {
        if (alliance == Alliance.BLUE) return blue;
        return new Pose(blue.getX(), 144.0 - blue.getY(), -blue.getHeading());
    }

    private void buildPaths() {
        follower.setStartingPose(mirror(startPose));

        s1 = follower.pathBuilder().addPath(new BezierLine(mirror(startPose), mirror(scorefirst)))
                .setLinearHeadingInterpolation(mirror(startPose).getHeading(), mirror(scorefirst).getHeading()).build();

        p1 = follower.pathBuilder().addPath(new BezierCurve(mirror(scorefirst), mirror(setFirstPick), mirror(firstPick)))
                .setLinearHeadingInterpolation(mirror(scorefirst).getHeading(), mirror(firstPick).getHeading()).build();

        s2 = follower.pathBuilder().addPath(new BezierLine(mirror(firstPick), mirror(scoreSecond)))
                .setLinearHeadingInterpolation(mirror(firstPick).getHeading(), mirror(scoreSecond).getHeading()).build();

        p2 = follower.pathBuilder().addPath(new BezierCurve(mirror(scoreSecond), mirror(setSecondPick), mirror(secondPick)))
                .setLinearHeadingInterpolation(mirror(scoreSecond).getHeading(), mirror(secondPick).getHeading()).build();

        s3 = follower.pathBuilder().addPath(new BezierCurve(mirror(secondPick), mirror(setThirdScore), mirror(thirdScore)))
                .setLinearHeadingInterpolation(mirror(secondPick).getHeading(), mirror(thirdScore).getHeading()).build();

        p3 = follower.pathBuilder().addPath(new BezierCurve(mirror(thirdScore), mirror(setThirdPick), mirror(thirdPick)))
                .setLinearHeadingInterpolation(mirror(thirdScore).getHeading(), mirror(thirdPick).getHeading()).build();

        s4 = follower.pathBuilder().addPath(new BezierLine(mirror(thirdPick), mirror(fourthScore)))
                .setLinearHeadingInterpolation(mirror(thirdPick).getHeading(), mirror(fourthScore).getHeading()).build();
    }

    @Override
    public void init() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        follower = r.follower;
        shooterSeq = new autoShooterSequence(r);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Y", mirror(startPose).getY());
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths(); // Build once at start
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        r.periodic();
        shooterSeq.update();

        switch (pathState) {
            case 0: follower.followPath(s1); pathState = 1; break;
            case 1: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 2; } break;
            case 2: if (pathTimer.getElapsedTimeSeconds() > 0.1) { shooterSeq.start(60); pathState = 3; } break;
            case 3: if (shooterSeq.isDone()) { shooterSeq.resetToIdle(); r.i.set(0.8); follower.followPath(p1); pathState = 4; } break;
            // ... Continue pattern for remaining paths ...
            case 4: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 5; } break;
            case 5: if (pathTimer.getElapsedTimeSeconds() > 0.65) { r.i.spinIdle(); follower.followPath(s2); pathState = 6; } break;
            case 6: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 7; } break;
            case 7: if (pathTimer.getElapsedTimeSeconds() > 0.1) { shooterSeq.start(60); pathState = 8; } break;
            case 8: if (shooterSeq.isDone()) { shooterSeq.resetToIdle(); r.i.set(0.8); follower.followPath(p2); pathState = 9; } break;
            case 9: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 10; } break;
            case 10: if (pathTimer.getElapsedTimeSeconds() > 0.65) { r.i.spinIdle(); follower.followPath(s3); pathState = 11; } break;
            case 11: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 12; } break;
            case 12: if (pathTimer.getElapsedTimeSeconds() > 0.1) { shooterSeq.start(60); pathState = 13; } break;
            case 13: if (shooterSeq.isDone()) { shooterSeq.resetToIdle(); r.i.set(0.8); follower.followPath(p3); pathState = 14; } break;
            case 14: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 15; } break;
            case 15: if (pathTimer.getElapsedTimeSeconds() > 0.65) { r.i.spinIdle(); follower.followPath(s4); pathState = 16; } break;
            case 16: if (!follower.isBusy()) { pathTimer.resetTimer(); pathState = 17; } break;
            case 17: if (pathTimer.getElapsedTimeSeconds() > 0.1) { shooterSeq.start(60); pathState = 18; } break;
            case 18: if (shooterSeq.isDone()) { r.s.stopMotor(); pathState = -1; } break;
        }
    }
}