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

@Autonomous(name = "ODOM Red Far", group = "Auto")
public class odoRedFar extends CommandOpMode {

    private NonVisionRobot r;

    // Coordinates MIRRORED for RED alliance
    // Mirror formula: x_red = 144 - x_blue, y stays same, heading = 180° - heading_blue
    private static final Pose START = new Pose(88, 8, Math.toRadians(90));
    private static final Pose SCORE_FIRST = new Pose(88, 15, Math.toRadians(72));

    private static final Pose SET_FIRST_PICK = new Pose(104, 36, Math.toRadians(0));
    private static final Pose FIRST_PICK = new Pose(138, 36, Math.toRadians(0));
    private static final Pose SCORE_SECOND = new Pose(88, 15, Math.toRadians(72));

    private static final Pose SET_SECOND_PICK = new Pose(135, 13, Math.toRadians(0));
    private static final Pose SECOND_PICK = new Pose(135, 6, Math.toRadians(0));
    private static final Pose THIRD_SCORE = new Pose(88, 15, Math.toRadians(72));

    private static final Pose PARK = new Pose(106, 12, Math.toRadians(90));

    @Override
    public void initialize() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.RED);

        // Set starting pose
        r.follower.setPose(START);

        // Background task: update odometry + telemetry
        schedule(new RunCommand(() -> {
            r.periodic();
            r.follower.update();

            Pose p = r.follower.getPose();
            telemetry.addData("═══════════════════════════", "");
            telemetry.addData("AUTO", "Red Far Side");
            telemetry.addData("Position", String.format("X: %.1f, Y: %.1f", p.getX(), p.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(p.getHeading())));
            r.s.getTelemetryData(telemetry);
            telemetry.addData("═══════════════════════════", "");
            telemetry.update();
        }));

        // Main auto sequence (same logic as blue, mirrored coordinates)
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> r.s.forDistance(90)),

                OdometryDrive.toPose(r, hardwareMap,
                        SCORE_FIRST.getX(), SCORE_FIRST.getY(),
                        Math.toDegrees(SCORE_FIRST.getHeading()), 0.5),
                new WaitCommand(2000),
                new Shoot(r),

                OdometryDrive.toPose(r, hardwareMap,
                        SET_FIRST_PICK.getX(), SET_FIRST_PICK.getY(),
                        Math.toDegrees(SET_FIRST_PICK.getHeading()), 0.6),

                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                FIRST_PICK.getX(), FIRST_PICK.getY(),
                                Math.toDegrees(FIRST_PICK.getHeading()), 0.4)
                ),

                OdometryDrive.toPose(r, hardwareMap,
                        SCORE_SECOND.getX(), SCORE_SECOND.getY(),
                        Math.toDegrees(SCORE_SECOND.getHeading()), 0.6),
                new InstantCommand(() -> r.s.forDistance(90)),
                new WaitCommand(2000),
                new Shoot(r),

                OdometryDrive.toPose(r, hardwareMap,
                        SET_SECOND_PICK.getX(), SET_SECOND_PICK.getY(),
                        Math.toDegrees(SET_SECOND_PICK.getHeading()), 0.6),

                new IntakeIn(r.i).alongWith(
                        OdometryDrive.toPose(r, hardwareMap,
                                SECOND_PICK.getX(), SECOND_PICK.getY(),
                                Math.toDegrees(SECOND_PICK.getHeading()), 0.4)
                ),

                OdometryDrive.toPose(r, hardwareMap,
                        THIRD_SCORE.getX(), THIRD_SCORE.getY(),
                        Math.toDegrees(THIRD_SCORE.getHeading()), 0.6),
                new InstantCommand(() -> r.s.forDistance(90)),
                new WaitCommand(2000),
                new Shoot(r),

                OdometryDrive.toPose(r, hardwareMap,
                        PARK.getX(), PARK.getY(),
                        Math.toDegrees(PARK.getHeading()), 0.5),

                new InstantCommand(r::stop)
        ));
    }

    @Override
    public void end() {
        if (r != null) r.stop();
    }
}