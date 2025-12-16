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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;

public class redClose extends CommandOpMode {

    private Follower follower;
    Robot r;
    Intake i;
    ShooterWait s;
    Webcam w;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    MultipleTelemetry multipleTelemetry;
    private final int RED_SCORE_ZONE_ID = 24;
    private final int BLUE_SCORE_ZONE_ID = 20;
    public static Pose startingPose;
    double distance = -1;
    private TelemetryManager telemetryM;
    private boolean slowModeActive = false;
    private double adjustSpeed = 0.25;
    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;
    private boolean isIntakeInward = false;
    private boolean isIntakeOutward = false;
    private boolean isShooterActive = false;
    private double dist = 0;
    private closePath close;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, Alliance.RED);
        r.follower = Constants.createFollower(hardwareMap);
        r.follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        r.follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        close = new closePath(r.follower, Alliance.RED);
        r.follower.setStartingPose(close.start);

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
                    telemetry.update();
                }),
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        new FollowPath(r.follower, close.scoreP())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> s.forDistance(dist))
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r.i)
                                .alongWith(
                                        new FollowPath(r.follower, close.next())
                                ),
                        new FollowPath(r.follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> s.forDistance(dist))
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r.i)
                                .alongWith(
                                        new FollowPath(r.follower, close.next())
                                ),
                        new FollowPath(r.follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> s.forDistance(dist))
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r.i)
                                .alongWith(
                                        new FollowPath(r.follower, close.next())
                                ),
                        new FollowPath(r.follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> s.forDistance(dist))
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r.i)
                                .alongWith(
                                        new FollowPath(r.follower, close.next())
                                ),
                        new FollowPath(r.follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> s.forDistance(dist))
                                                )
                                ),
                        new Shoot(r)
                )
        );
    }

}
