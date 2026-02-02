package org.firstinspires.ftc.teamcode.decode_auto.odo_auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.OdometryDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;

@Autonomous(name = "ODOM Blue Far", group = "Auto")
public class odoBlueFar extends CommandOpMode {

    private NonVisionRobot r;

    // TODO: MEASURE THESE COORDINATES ON YOUR FIELD!
    // Far side starting position (opposite side from close)
    private static final Pose START = new Pose(12, 12, Math.toRadians(0));  // TODO: Measure!

    // Scoring positions (probably similar to close, but from other side)
    private static final Pose SCORE_FIRST = new Pose(45, 100, Math.toRadians(136));  // TODO: Measure!

    // Sample pickup positions (far side samples)
    private static final Pose FIRST_PICK = new Pose(120, 84, Math.toRadians(0));  // TODO: Measure!
    private static final Pose SCORE_SECOND = new Pose(45, 100, Math.toRadians(136));

    private static final Pose SECOND_PICK = new Pose(120, 60, Math.toRadians(0));  // TODO: Measure!
    private static final Pose THIRD_SCORE = new Pose(45, 100, Math.toRadians(136));

    private static final Pose THIRD_PICK = new Pose(120, 36, Math.toRadians(0));  // TODO: Measure!
    private static final Pose FOURTH_SCORE = new Pose(45, 100, Math.toRadians(136));

    // Park position
    private static final Pose PARK = new Pose(60, 60, Math.toRadians(90));  // TODO: Measure!

    @Override
    public void initialize() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);

        // Set starting pose
        r.follower.setPose(START);

        // Background task: update odometry + telemetry
        schedule(new RunCommand(() -> {
            r.periodic();
            r.follower.update();

            Pose p = r.follower.getPose();
            telemetry.addData("═══════════════════════════", "");
            telemetry.addData("AUTO", "Blue Far Side");
            telemetry.addData("Position", String.format("X: %.1f, Y: %.1f", p.getX(), p.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(p.getHeading())));
            r.s.getTelemetryData(telemetry);
            telemetry.addData("═══════════════════════════", "");
            telemetry.update();
        }));

        // Main auto sequence
        schedule(new SequentialCommandGroup(
                // ═══════════════════════════════════════════════
                // SCORE PRELOADED SAMPLES (3 shots)
                // ═══════════════════════════════════════════════

                // Start spinning shooter early
                new InstantCommand(() -> r.s.forDistance(90)),  // Far shots need more power

                // Drive to scoring position (long drive from far side)
                OdometryDrive.toPose(r, hardwareMap,
                        SCORE_FIRST.getX(),
                        SCORE_FIRST.getY(),
                        Math.toDegrees(SCORE_FIRST.getHeading()),
                        0.7),  // Faster for long drive

                // Wait for shooter to spin up
                new WaitCommand(2000),  // Longer wait for far shot spinup

                // Shoot 3 preloaded samples
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK SAMPLE 1 + SCORE
                // ═══════════════════════════════════════════════

                // Drive to first sample while intaking
                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                FIRST_PICK.getX(),
                                FIRST_PICK.getY(),
                                Math.toDegrees(FIRST_PICK.getHeading()),
                                0.6)
                ),

                // Drive back to scoring position
                OdometryDrive.toPose(r, hardwareMap,
                        SCORE_SECOND.getX(),
                        SCORE_SECOND.getY(),
                        Math.toDegrees(SCORE_SECOND.getHeading()),
                        0.7),

                // Spin up and shoot
                new InstantCommand(() -> r.s.forDistance(90)),
                new WaitCommand(2000),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK SAMPLE 2 + SCORE
                // ═══════════════════════════════════════════════

                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                SECOND_PICK.getX(),
                                SECOND_PICK.getY(),
                                Math.toDegrees(SECOND_PICK.getHeading()),
                                0.6)
                ),

                OdometryDrive.toPose(r, hardwareMap,
                        THIRD_SCORE.getX(),
                        THIRD_SCORE.getY(),
                        Math.toDegrees(THIRD_SCORE.getHeading()),
                        0.7),

                new InstantCommand(() -> r.s.forDistance(90)),
                new WaitCommand(2000),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK SAMPLE 3 + SCORE (if time permits)
                // ═══════════════════════════════════════════════

                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                THIRD_PICK.getX(),
                                THIRD_PICK.getY(),
                                Math.toDegrees(THIRD_PICK.getHeading()),
                                0.6)
                ),

                OdometryDrive.toPose(r, hardwareMap,
                        FOURTH_SCORE.getX(),
                        FOURTH_SCORE.getY(),
                        Math.toDegrees(FOURTH_SCORE.getHeading()),
                        0.7),

                new InstantCommand(() -> r.s.forDistance(90)),
                new WaitCommand(2000),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PARK
                // ═══════════════════════════════════════════════

                OdometryDrive.toPose(r, hardwareMap,
                        PARK.getX(),
                        PARK.getY(),
                        Math.toDegrees(PARK.getHeading()),
                        0.6),

                // ═══════════════════════════════════════════════
                // CLEAN SHUTDOWN
                // ═══════════════════════════════════════════════
                new InstantCommand(r::stop)
        ));
    }

    @Override
    public void end() {
        if (r != null) r.stop();
    }
}