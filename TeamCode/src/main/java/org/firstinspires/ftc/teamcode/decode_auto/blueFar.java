package org.firstinspires.ftc.teamcode.decode_auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.paths.farPaths;

@Autonomous(name = "Blue Far Auto", group = "Auto")
public class blueFar extends CommandOpMode {

    private NonVisionRobot r;
    private farPaths far;

    @Override
    public void initialize() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        far = new farPaths(r.follower, Alliance.BLUE);

        r.follower.setStartingPose(far.start);
        r.follower.update();

        // Background task: update robot + telemetry (NO REQUIREMENTS)
        schedule(new RunCommand(() -> {
            r.periodic();

            Pose p = r.follower.getPose();
            if (p != null) {
                telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
                        p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
            }
            telemetry.addData("Shooter Target", "%.1f", r.s.getTarget());
            telemetry.addData("Shooter Vel", "%.1f", r.s.getVelocity());
            telemetry.addData("At Target?", r.s.isAtVelocity(r.s.getTarget()) ? "YES" : "NO");
            r.s.getTelemetryData(telemetry);
            telemetry.update();
        }));

        // Main auto sequence
        schedule(new SequentialCommandGroup(
                // ═══════════════════════════════════════════════
                // SCORE 1 (Pre-loaded samples)
                // ═══════════════════════════════════════════════
                new FollowPath(r.follower, far.scoreP())
                        .withTTrigger(0.01, () -> {
                            // Hardcoded FAR distance (>80 triggers spinFar())
                            r.s.forDistance(90);
                        }),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK 1 + SCORE 2
                // ═══════════════════════════════════════════════
                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, far.pickOne())),
                new FollowPath(r.follower, far.scoreTwo())
                        .withTTrigger(0.01, () -> {
                            r.s.forDistance(90);
                        }),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PICK 2 + SCORE 3
                // ═══════════════════════════════════════════════
                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, far.pickTwo())),
                new FollowPath(r.follower, far.scoreThird())
                        .withTTrigger(0.01, () -> {
                            r.s.forDistance(90);
                        }),
                new Shoot(r),

                // ═══════════════════════════════════════════════
                // PARK
                // ═══════════════════════════════════════════════
                new FollowPath(r.follower, far.parkPath()),

                // ═══════════════════════════════════════════════
                // CLEAN SHUTDOWN
                // ═══════════════════════════════════════════════
                new InstantCommand(() -> {
                    r.i.spinIdle();
                    r.s.feedZero();
                    r.s.stopMotor();
                    r.stop();
                })
        ));
    }

    @Override
    public void end() {
        if (r != null) r.stop();
    }
}