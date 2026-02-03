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

@Autonomous(name = "ODOM Blue Close", group = "Auto")
public class odoBlueClose extends CommandOpMode {

    private NonVisionRobot r;

    // Coordinates from your closePath.java (BLUE alliance)
    private static final Pose START = new Pose(21.913, 123, Math.toRadians(136));
    private static final Pose SCORE_FIRST = new Pose(45, 100, Math.toRadians(136));

    private static final Pose FIRST_PICK = new Pose(16.5, 84, Math.toRadians(180));
    private static final Pose SCORE_SECOND = new Pose(45, 100, Math.toRadians(136));

    private static final Pose SECOND_PICK = new Pose(10, 60, Math.toRadians(180));
    private static final Pose THIRD_SCORE = new Pose(45, 100, Math.toRadians(136));

    private static final Pose THIRD_PICK = new Pose(10, 36, Math.toRadians(180));
    private static final Pose FOURTH_SCORE = new Pose(45, 100, Math.toRadians(136));

    @Override
    public void initialize() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);

        // Set starting pose - CRITICAL for odometry accuracy!
        r.follower.setPose(START);

        // Background task: update odometry + telemetry
        schedule(new RunCommand(() -> {
            r.periodic();
            r.follower.update();  // Update odometry every loop!

            Pose p = r.follower.getPose();
            telemetry.addData("═══════════════════════════", "");
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
                new InstantCommand(() -> r.s.forDistance(60)),

                // Drive to scoring position
                OdometryDrive.toPose(r, hardwareMap,
                        SCORE_FIRST.getX(),
                        SCORE_FIRST.getY(),
                        Math.toDegrees(SCORE_FIRST.getHeading()),
                        0.6),

                // Wait for shooter to spin up
                new WaitCommand(1500),

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
                                0.5)
                ),

                // Drive back to scoring position
                OdometryDrive.toPose(r, hardwareMap,
                        SCORE_SECOND.getX(),
                        SCORE_SECOND.getY(),
                        Math.toDegrees(SCORE_SECOND.getHeading()),
                        0.6),

                // Spin up and shoot
                new InstantCommand(() -> r.s.forDistance(60)),
                new WaitCommand(1500),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK SAMPLE 2 + SCORE
                // ═══════════════════════════════════════════════

                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                SECOND_PICK.getX(),
                                SECOND_PICK.getY(),
                                Math.toDegrees(SECOND_PICK.getHeading()),
                                0.5)
                ),

                OdometryDrive.toPose(r, hardwareMap,
                        THIRD_SCORE.getX(),
                        THIRD_SCORE.getY(),
                        Math.toDegrees(THIRD_SCORE.getHeading()),
                        0.6),

                new InstantCommand(() -> r.s.forDistance(60)),
                new WaitCommand(1500),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK SAMPLE 3 + SCORE (if time permits)
                // ═══════════════════════════════════════════════

                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                THIRD_PICK.getX(),
                                THIRD_PICK.getY(),
                                Math.toDegrees(THIRD_PICK.getHeading()),
                                0.5)
                ),

                OdometryDrive.toPose(r, hardwareMap,
                        FOURTH_SCORE.getX(),
                        FOURTH_SCORE.getY(),
                        Math.toDegrees(FOURTH_SCORE.getHeading()),
                        0.6),

                new InstantCommand(() -> r.s.forDistance(60)),
                new WaitCommand(1500),
                new Shoot(r),

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