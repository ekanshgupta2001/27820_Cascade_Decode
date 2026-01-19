package org.firstinspires.ftc.teamcode.decode_auto;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Alliance;

import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.paths.closePath;

public class blueClose extends CommandOpMode {

    private NonVisionRobot r;
    private closePath close;

    @Override
    public void initialize() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        close = new closePath(r.follower, Alliance.BLUE);

        // Always use the path file start (unless you really need override)
        r.follower.setStartingPose(close.start);
        r.follower.update();

        // 1) Forever loop: update robot + telemetry (NO REQUIREMENTS)
        schedule(new RunCommand(() -> {
            r.periodic();

            Pose p = r.follower.getPose();
            if (p != null) {
                telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
                        p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
            }
            telemetry.update();
        }));

        // 2) Main auto sequence (deterministic, no next())
        schedule(new SequentialCommandGroup(
                // Score 1
                new FollowPath(r.follower, close.scoreP())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 60); // starts spinup early
                            }
                        }),
                new Shoot(r),

                // Pick 1 + Score 2
                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.pickOne())),
                new FollowPath(r.follower, close.scoreTwo())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 60); // starts spinup early
                            }
                        }),
                new Shoot(r),

                // Pick 2 + Score 3
                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.pickTwo())),
                new FollowPath(r.follower, close.scoreThird())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 60); // starts spinup early
                            }
                        }),
                new Shoot(r),

                // Pick 3 + Score 4 (optional, you had it in paths)
                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.pickThree())),
                new FollowPath(r.follower, close.scoreFourth())
                        .withTTrigger(0.40, () -> {
                            Pose p = r.follower.getPose();
                            Pose target = r.getShootTarget();
                            if (p != null && target != null) {
                                double dx = Math.abs(target.getX() - p.getX());
                                double dy = Math.abs(target.getY() - p.getY());
                                r.s.forDistance(dx, 60); // starts spinup early
                            }
                        }),
                new Shoot(r),

                // End clean
                new InstantCommand(r::stop)
        ));
    }

    @Override
    public void end() {
        if (r != null) r.stop();
    }
}