package org.firstinspires.ftc.teamcode.decode_auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Alliance;

import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.paths.closePath;

public class redClose extends CommandOpMode {

    NonVisionRobot r;
    public static Pose startingPose;

    // Global distance variables for telemetry and shooter sync
    public double dist_x = 0.0;
    public double dist_y = 0.0;

    private closePath close;

    @Override
    public void initialize() {
        // 1. Initialize Robot (Internal Follower and Subsystems created here)
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);

        // 2. Initialize Paths
        close = new closePath(r.follower, Alliance.BLUE);

        // 3. Set Starting Pose (prioritize the path file's start position)
        Pose actualStart = (startingPose == null) ? close.start : startingPose;
        r.follower.setStartingPose(actualStart);
        r.follower.update();

        // 4. Command Scheduling
        schedule(
                // This replaces the manual follower.update() and handles bulk caching
                new RunCommand(r::periodic),

                // Telemetry loop for monitoring distances
                new RunCommand(() -> {
                    Pose p = r.follower.getPose();
                    if (p != null) {
                        telemetry.addData("Robot X", p.getX());
                        telemetry.addData("Robot Y", p.getY());
                    }
                    telemetry.addData("Target Dist X", dist_x);
                    telemetry.addData("Target Dist Y", dist_y);
                    telemetry.update();
                }),

                // Main Autonomous Sequence
                new SequentialCommandGroup(
                        new WaitCommand(500),

                        // --- Path to Score 1 ---
                        new FollowPath(r.follower, close.scoreP())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(new InstantCommand(this::updateShooterDistances))
                                ),
                        new Shoot(r, dist_x, dist_y),

                        // --- Path to Intake & Score 2 ---
                        new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.next())),
                        new FollowPath(r.follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(new InstantCommand(this::updateShooterDistances))
                                ),
                        new Shoot(r, dist_x, dist_y),

                        // --- Path to Intake & Score 3 ---
                        new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.next())),
                        new FollowPath(r.follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(new InstantCommand(this::updateShooterDistances))
                                ),
                        new Shoot(r, dist_x, dist_y)
                )
        );
    }

    /**
     * Calculates the X and Y components of the distance to the target
     * and passes them to the ShooterWait subsystem.
     */
    private void updateShooterDistances() {
        Pose robotPose = r.follower.getPose();
        Pose targetPose = r.getShootTarget(); // Gets the mirrored or standard pose

        if (robotPose != null && targetPose != null) {
            // Absolute distance in inches for the shooter's calculation
            dist_x = Math.abs(targetPose.getX() - robotPose.getX());
            dist_y = Math.abs(targetPose.getY() - robotPose.getY());

            // Pass the calculated distances to your ShooterWait subsystem
            r.s.forDistance(dist_x, dist_y);
        }
    }

    @Override
    public void end() {
        if (r != null) r.stop();
    }
}
