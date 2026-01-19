package org.firstinspires.ftc.teamcode.decode_auto;

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

public class blueFar extends CommandOpMode {

    private NonVisionRobot r;
    private farPaths far;

    @Override
    public void initialize() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        far = new farPaths(r.follower, Alliance.BLUE);

        r.follower.setStartingPose(far.start);
        r.follower.update();

        // Forever periodic + telemetry (no requirements)
        schedule(new RunCommand(() -> {
            r.periodic();
            Pose p = r.follower.getPose();
            telemetry.addData("Pose", p);
            telemetry.addData("Shooter Target", r.s.getTarget());
            telemetry.addData("Shooter Vel", r.s.getVelocity());
            telemetry.addData("At Target?", r.s.isAtVelocity(r.s.getTarget()));
            telemetry.update();
        }));

        // Main auto
        schedule(new SequentialCommandGroup(
                // Score 1
                new FollowPath(r.follower, far.scoreP())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 90);
                            }
                        }),
                new Shoot(r),

// ... later ...

                new FollowPath(r.follower, far.scoreTwo())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 90);
                            }
                        }),
                new Shoot(r),

// ... later ...

                new FollowPath(r.follower, far.scoreThird())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 90);
                            }
                        }),
                new Shoot(r),

                // Park
                new FollowPath(r.follower, far.parkPath()),

                // End safe
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