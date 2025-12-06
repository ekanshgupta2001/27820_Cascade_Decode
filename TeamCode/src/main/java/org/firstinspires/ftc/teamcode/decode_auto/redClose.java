package org.firstinspires.ftc.teamcode.decode_auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
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
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.paths.closePath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;

public class redClose extends CommandOpMode {

    private Follower follower;
    Intake i;
    Shooter s;
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
    private closePath close;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        w = new Webcam(hardwareMap, telemetry, "Webcam 1");
        i = new Intake(hardwareMap);
        s = new Shooter(hardwareMap, i);

        close = new closePath(follower, Alliance.RED);

        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        Shoot shootCommand = new Shoot(s, i);
        shootCommand.follower = this.follower;

        IntakeIn intakeCommand = new IntakeIn(i);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(s::periodic),
                new RunCommand(w::periodic),
                new RunCommand(i::periodic),
                new RunCommand(() -> {
                    telemetry.addData("Pose", follower.getPose());
                    telemetry.addData("Follower Busy", follower.isBusy());
                    telemetry.addData("Shooter At Target Velocity: ", s.isAtVelocity(1000));
                    telemetry.update();
                }),
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        new FollowPath(follower, close.scoreP())
                                .alongWith(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        s.spinCloseCommand()
                                                )
                                ),
                        shootCommand,
                        intakeCommand
                                .alongWith(
                                        new FollowPath(follower, close.next())
                                ),
                        new FollowPath(follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        s.spinCloseCommand()
                                                )
                                ),
                        shootCommand,
                        intakeCommand
                                .alongWith(
                                        new FollowPath(follower, close.next())
                                ),
                        new FollowPath(follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        s.spinCloseCommand()
                                                )
                                ),
                        shootCommand,
                        intakeCommand
                                .alongWith(
                                        new FollowPath(follower, close.next())
                                ),
                        new FollowPath(follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        s.spinCloseCommand()
                                                )
                                ),
                        shootCommand,
                        intakeCommand
                                .alongWith(
                                        new FollowPath(follower, close.next())
                                ),
                        new FollowPath(follower, close.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        s.spinCloseCommand()
                                                )
                                ),
                        shootCommand

                )
        );
    }

}
