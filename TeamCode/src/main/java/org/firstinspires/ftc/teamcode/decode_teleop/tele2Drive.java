package org.firstinspires.ftc.teamcode.decode_teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;

//@TeleOp
@Config
public class tele2Drive extends OpMode {
    private Follower follower;
    Intake i;
    Shooter s;
    Webcam w;
    private Servo indicatorLight;
    private Servo indicatorLight2;
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

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        w = new Webcam(hardwareMap, telemetry, "Webcam 1");
        i = new Intake(hardwareMap);
        s = new Shooter(hardwareMap, i);

        indicatorLight = hardwareMap.get(Servo.class, "IL");
        indicatorLight2 = hardwareMap.get(Servo.class, "ILTwo");

        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void init_loop(){
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            w.setTargetTagID(BLUE_SCORE_ZONE_ID);
            telemetry.addLine("Blue Tag Found");
            indicatorLight.setPosition(0.87);
            indicatorLight2.setPosition(0.87);

        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            w.setTargetTagID(RED_SCORE_ZONE_ID);
            telemetry.addLine("Red Tag Found");
            indicatorLight.setPosition(0.61);
            indicatorLight2.setPosition(0.61);

        }
    }

    @Override
    public void start(){
        indicatorLight.setPosition(-0.95);
        indicatorLight2.setPosition(-0.95);
        follower.startTeleopDrive();
        CommandScheduler.getInstance().schedule(s.feedUpCommand());
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        follower.update();
        telemetryM.update();

        hood();
        intake();
        drive();

        lastRightBumperState = operatorGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        lastLeftBumperState = operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER);

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            isShooterActive = true;
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)){
            CommandScheduler.getInstance().schedule(s.stop());
            CommandScheduler.getInstance().schedule(s.stopCommand());
            isShooterActive = false;
        }

        w.displayTagTelemetry(w.getTargetTag());
        s.getTelemetryData(telemetry);
        i.getTelemetryData(telemetry);
        telemetry.addData("Distance to April Tag", distance);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }

    public void drive(){
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            slowModeActive = true;
            telemetry.addLine("Slow Mode On");
            indicatorLight.setPosition(-0.29);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            slowModeActive = false;
            telemetry.addLine("Slow Mode off");
            indicatorLight.setPosition(-0.83);
        }

        if (!slowModeActive)
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

        if (slowModeActive)
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * adjustSpeed,
                    -gamepad1.left_stick_x * adjustSpeed,
                    -gamepad1.right_stick_x * adjustSpeed,
                    true
            );

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && adjustSpeed <= 1.0) {
            adjustSpeed += 0.2;
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) && adjustSpeed >= 0.0) {
            adjustSpeed -= 0.2;
        }

        telemetry.addLine("Should move");
    }

    public void intake(){
        boolean currentRightBumper = operatorGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        boolean currentLeftBumper = operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER);

        if (currentRightBumper && currentLeftBumper) {
            CommandScheduler.getInstance().schedule(i.stopCommand());
            isIntakeInward = false;
            isIntakeOutward = false;
        } else if (currentRightBumper && !lastRightBumperState) {
            isIntakeInward = !isIntakeInward;
            isIntakeOutward = false;
        } else if (currentLeftBumper && !lastLeftBumperState) {
            isIntakeOutward = !isIntakeOutward;
            isIntakeInward = false;
        }

        if (isIntakeInward) {
            CommandScheduler.getInstance().schedule(i.inCommand());
            if (!isShooterActive){
                CommandScheduler.getInstance().schedule(s.intakein());
            }
        } else if (isIntakeOutward) {
            CommandScheduler.getInstance().schedule(i.outCommand());
        } else {
            if (i.getDefaultCommand() == null) {
                i.setDefaultCommand(i.idleCommand());
            }
        }
    }

    public void hood(){
        w.periodic();
        int currentID = w.getTargetTagID();
//        if (currentID >= 0){
//            distance = w.getDistancetoTagId();
//        }
        if (isShooterActive) {
            if (distance >= 200) {
//                CommandScheduler.getInstance().schedule(s.feedUpCommand());
                CommandScheduler.getInstance().schedule(s.scoreFarCommand());
                indicatorLight2.setPosition(0.57);
            }
            else {
                telemetry.addLine("Score Close active");
                CommandScheduler.getInstance().schedule(s.scoreCloseCommand());
                indicatorLight2.setPosition(0.81);
            }
        }
    }


}
