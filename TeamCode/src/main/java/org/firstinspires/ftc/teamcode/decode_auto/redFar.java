package org.firstinspires.ftc.teamcode.decode_auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.paths.farPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class redFar extends CommandOpMode {
    NonVisionRobot r;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    public double dist_x = 0.0;
    public double dist_y = 0.0;
    private Pose targetPose;
    private farPaths far;

    @Override
    public void initialize() {
        // 1. Core Initialization
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.RED);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // 2. Setup Target and Paths
        targetPose = r.getShootTarget();
        far = new farPaths(r.follower, Alliance.RED);

        // 3. Pose Setup
        Pose actualStart = (startingPose == null) ? far.start : startingPose;
        r.follower.setStartingPose(actualStart);
        r.follower.update();

        schedule(
                new RunCommand(r::periodic),

                // Continuous calculation and telemetry
                new RunCommand(() -> {
                    updateShooterDistances();
                    telemetry.addData("Pose", r.follower.getPose());
                    telemetry.addData("Dist X", dist_x);
                    telemetry.addData("Dist Y", dist_y);
                    telemetry.addData("Shooter At Velocity", r.s.isAtVelocity(1000));
                    telemetry.update();
                }),

                new SequentialCommandGroup(
                        new WaitCommand(1),

                        // --- First Score ---
                        new FollowPath(r.follower, far.scoreP())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(new InstantCommand(this::updateShooterDistances)) // Added missing )
                                ),
                        new Shoot(r),

                        // --- Intake and Second Score ---
                        new IntakeIn(r.i),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(new InstantCommand(this::updateShooterDistances)) // Added missing )
                                ),
                        new Shoot(r),

                        // --- Intake and Third Score ---
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(new InstantCommand(this::updateShooterDistances)) // Added missing )
                                ),
                        new Shoot(r),

                        // --- Cleanup ---
                        new FollowPath(r.follower, far.next()),
                        new InstantCommand(() -> {
                            r.i.idleCommand();
                            r.s.feedZero();
                            r.s.stopMotor();
                        })
                )
        );
    }

    /**
     * Helper to calculate dist_x/y and update shooter subsystem
     */
    private void updateShooterDistances() {
        Pose robotPose = r.follower.getPose();
        if (robotPose != null && targetPose != null) {
            dist_x = Math.abs(targetPose.getX() - robotPose.getX());
            dist_y = Math.abs(targetPose.getY() - robotPose.getY());
            r.s.forDistance(dist_x, dist_y);
        }
    }

    @Override
    public void end() {
        if (r != null) r.stop();
    }
}
