//package org.firstinspires.ftc.teamcode.decode_auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//
//import com.pedropathing.geometry.Pose;
//
//import org.firstinspires.ftc.teamcode.Alliance;
//import org.firstinspires.ftc.teamcode.NonVisionRobot;
//import org.firstinspires.ftc.teamcode.commands.FollowPath;
//import org.firstinspires.ftc.teamcode.commands.IntakeIn;
//import org.firstinspires.ftc.teamcode.commands.Shoot;
//import org.firstinspires.ftc.teamcode.paths.closePath;
//
//@Autonomous(name = "Red Close Auto", group = "Auto")
//public class redClose extends CommandOpMode {
//
//    private NonVisionRobot r;
//    private closePath close;
//
//    @Override
//    public void initialize() {
//        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.RED);
//        close = new closePath(r.follower, Alliance.RED);
//
//        r.follower.setStartingPose(close.start);
//        r.follower.update();
//
//        // Background task: update robot + telemetry (NO REQUIREMENTS)
//        schedule(new RunCommand(() -> {
//            r.periodic();
//
//            Pose p = r.follower.getPose();
//            if (p != null) {
//                telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
//                        p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
//            }
//            r.s.getTelemetryData(telemetry);
//            telemetry.update();
//        }));
//
//        // Main auto sequence
//        schedule(new SequentialCommandGroup(
//                // ═══════════════════════════════════════════════
//                // SCORE 1 (Pre-loaded samples)
//                // ═══════════════════════════════════════════════
//                new FollowPath(r.follower, close.scoreP())
//                        .withTTrigger(0.01, () -> {
//                            // Hardcoded expected distance for early spinup
//                            r.s.forDistance(60); // Medium distance expected
//                        }),
//                new Shoot(r),  // Will recalculate actual distance when it starts
//
//                // ═══════════════════════════════════════════════
//                // PICK 1 + SCORE 2
//                // ═══════════════════════════════════════════════
//                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.pickOne())),
//                new FollowPath(r.follower, close.scoreTwo())
//                        .withTTrigger(0.01, () -> {
//                            r.s.forDistance(60); // Medium distance expected
//                        }),
//                new Shoot(r),
//
//                // ═══════════════════════════════════════════════
//                // PICK 2 + SCORE 3
//                // ═══════════════════════════════════════════════
//                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.pickTwo())),
//                new FollowPath(r.follower, close.scoreThird())
//                        .withTTrigger(0.01, () -> {
//                            r.s.forDistance(60); // Medium distance expected
//                        }),
//                new Shoot(r),
//
//                // ═══════════════════════════════════════════════
//                // PICK 3 + SCORE 4
//                // ═══════════════════════════════════════════════
//                new IntakeIn(r.i).alongWith(new FollowPath(r.follower, close.pickThree())),
//                new FollowPath(r.follower, close.scoreFourth())
//                        .withTTrigger(0.01, () -> {
//                            r.s.forDistance(60); // Medium distance expected
//                        }),
//                new Shoot(r),
//
//                // ═══════════════════════════════════════════════
//                // CLEAN SHUTDOWN
//                // ═══════════════════════════════════════════════
//                new InstantCommand(r::stop)
//        ));
//    }
//
//    @Override
//    public void end() {
//        if (r != null) r.stop();
//    }
//}