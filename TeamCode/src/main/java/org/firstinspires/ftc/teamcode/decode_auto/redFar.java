package org.firstinspires.ftc.teamcode.decode_auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.paths.closePath;
import org.firstinspires.ftc.teamcode.paths.farPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;

public class redFar extends CommandOpMode {
    Robot r;
    public static Pose startingPose;
    MultipleTelemetry multipleTelemetry;
    private TelemetryManager telemetryM;
    private double dist = 0;
    private farPaths far;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, Alliance.RED);
        r.follower = Constants.createFollower(hardwareMap);
        r.follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        r.follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        far = new farPaths(r.follower, Alliance.RED);
        r.follower.setStartingPose(far.start);

        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        schedule(
                new RunCommand(r::periodic),
                new RunCommand(() -> {
                    dist = r.getShootTarget().distanceFrom(r.follower.getPose());
                    r.s.forDistance(dist);
                }),
                new RunCommand(() -> {
                    telemetry.addData("Pose", r.follower.getPose());
                    telemetry.addData("Follower Busy", r.follower.isBusy());
                    telemetry.addData("Shooter At Target Velocity: ", r.s.isAtVelocity(1000));
                    telemetry.update();
                }),
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        new FollowPath(r.follower, far.scoreP())
                                .alongWith(
                                        new RunCommand(r.i::idleCommand),
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.forDistance(dist))
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r.i),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next())
                                .alongWith(
                                        new RunCommand(r.i::idleCommand),
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.forDistance(dist))
                                                )

                                ),
                        new Shoot(r),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next()),
                        new FollowPath(r.follower, far.next())
                                .alongWith(
                                        new RunCommand(r.i::idleCommand),
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.forDistance(dist))
                                                )

                                ),
                        new Shoot(r),
                        new FollowPath(r.follower, far.next()),
                        new RunCommand(r.i::idleCommand),
                        new RunCommand(r.s::feedZero),
                        new RunCommand(r.s::stopMotor)
                )
        );
    }

    @Override
    public void end(){
        r.stop();
    }

}
