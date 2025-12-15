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

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.util.Timer;

@TeleOp
@Config
public class tele2DriveTest extends OpMode {
    Robot r;
    private Servo indicatorLight;
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
    private final Timer shoot = new Timer();
    private final Timer kick = new Timer();
    private int shootState = 0;
    public double dist;

    @Override
    public void init() {
        r = new Robot(hardwareMap, telemetry, Alliance.BLUE);

        indicatorLight = hardwareMap.get(Servo.class, "IL");


        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void init_loop() {
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.w.setTargetTagID(BLUE_SCORE_ZONE_ID);
            r.setAlliance(Alliance.BLUE);
            telemetry.addLine("Blue Tag Found");
            indicatorLight.setPosition(0.87);


        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.w.setTargetTagID(RED_SCORE_ZONE_ID);
            r.setAlliance(Alliance.RED);
            telemetry.addLine("Red Tag Found");
            indicatorLight.setPosition(0.61);


        }
    }

    @Override
    public void start() {
        indicatorLight.setPosition(-0.95);
        r.periodic();
        r.follower.startTeleopDrive();
        r.setShootTarget();
        gamepad1.rumbleBlips(1);
        gamepad2.rumbleBlips(1);
    }

    @Override
    public void loop() {
        r.periodic();

        CommandScheduler.getInstance().run();
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        telemetryM.update();

        hood();
        intake();
        drive();

        lastRightBumperState = operatorGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        lastLeftBumperState = operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER);

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            isShooterActive = true;
            shootState = 0;
            shoot.resetTimer();
            gamepad2.rumbleBlips(1);
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            r.s.stopMotor();
            isShooterActive = false;
            shootState = 0;
            shoot.resetTimer();
            gamepad2.rumbleBlips(2);
        }

        r.w.displayTagTelemetry(r.w.getTargetTag());
        r.s.getTelemetryData(telemetry);
        r.i.getTelemetryData(telemetry);
        telemetry.addData("Distance to April Tag", dist);
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }

    public void drive() {
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            telemetry.addLine("Slow Mode On");
            indicatorLight.setPosition(-0.29);
            gamepad1.rumbleBlips(1);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slowModeActive = false;
            telemetry.addLine("Slow Mode off");
            indicatorLight.setPosition(-0.83);
            gamepad1.rumbleBlips(2);
        }

        if (!slowModeActive)
            r.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

        if (slowModeActive)
            r.follower.setTeleOpDrive(
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

    public void intake() {
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            isIntakeInward = !isIntakeInward;
            isIntakeOutward = false;
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            isIntakeOutward = !isIntakeOutward;
            isIntakeInward = false;
        }
        if (isIntakeInward) {

            CommandScheduler.getInstance().schedule(r.i.inCommand());
        } else if (isIntakeOutward) {

            CommandScheduler.getInstance().schedule(r.i.outCommand());
        } else {
            if (r.i.getDefaultCommand() == null) {
                r.i.setDefaultCommand(r.i.idleCommand());
            }
        }
    }

    public void hood() {
        int currentID = r.w.getTargetTagID();
        if (currentID >= 0) {
            distance = r.w.getDistancetoTagId();
        }

        if (isShooterActive && r.w.getTargetTag() != null) {
            CommandScheduler.getInstance().schedule(r.i.idleCommand());
            dist = r.getShootTarget().distanceFrom(r.follower.getPose());
            r.s.forDistance(dist);

            if (shootState == 0) {
                if (shoot.getElapsedTime() > 0.8) {
                    shootState = 1;
                    kick.resetTimer();
                }
            } else if (shootState == 1 || shootState == 2 || shootState == 3) {
                r.i.set(0.0);
                r.s.kickUp();
                if (kick.getElapsedTime() > 0.5) {
                    r.s.kickDown();
                    if (kick.getElapsedTime() > 0.8){
                        r.i.set(-0.28);
                    }
                    if (shootState == 3) {
                        shootState = 4;
                    } else {
                        shootState++;
                        shoot.resetTimer();
                    }
                }
            }

        }
    }

}

