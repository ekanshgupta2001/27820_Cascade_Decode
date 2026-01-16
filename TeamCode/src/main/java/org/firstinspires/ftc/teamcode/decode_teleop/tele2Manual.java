package org.firstinspires.ftc.teamcode.decode_teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.sun.source.tree.IfTree;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.NonVisionRobot;

@TeleOp(name = "tele2NoVision")
@Config
public class tele2Manual extends OpMode {

    NonVisionRobot r;

    // PWM light on a servo port -> map as Servo (0..1 position).
    private Servo indicatorLight;
    private double currentLightPos = 0.0;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    private TelemetryManager telemetryM;
    private MultipleTelemetry multipleTelemetry;

    private boolean slowModeActive = false;
    private double adjustSpeed = 0.25;

    private boolean isIntakeInward = false;
    private boolean isIntakeOutward = false;

    // ----------------------------
    // Shooter modes
    // ----------------------------
    private enum ShooterMode { AUTO, MANUAL }
    private ShooterMode shooterMode = ShooterMode.MANUAL;

    // AUTO firing state
    private boolean autoShooterActive = false;
    private final Timer shootTimer = new Timer();

    // Pose-calibration flag
    private boolean calibrated = false;

    // Distance used for shooter calculations
    public double dist = 0.0;

    Pose targetPose;

    // Put your REAL Pedro field coordinates here for the "top triangle" landmark.
    // Heading should be in RADIANS, and should match how you want the robot oriented
    // right after calibration.
    private static final Pose BLUE_TOP_TRIANGLE_POSE = new Pose(
            72, 72, 0 // TODO: replace with real x,y,heading(rad)
    );

    // A safe default starting pose (mirrors for red).
    private static final Pose BLUE_START_POSE = new Pose(
            12, 12, 0 // TODO: replace with your real teleop start pose
    );

    @Override
    public void init() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);

        indicatorLight = hardwareMap.get(Servo.class, "IL");

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        setLightPos(0.0);
    }

    @Override
    public void init_loop() {
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        // Alliance select (no webcam)
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.setAlliance(Alliance.BLUE);
            setLightPos(0.6);
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.setAlliance(Alliance.RED);
            setLightPos(0.6);
        }

        telemetry.addData("Alliance", r.a);
        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Calibrated?", calibrated);
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startPose = (r.a == Alliance.RED) ? BLUE_START_POSE.mirror() : BLUE_START_POSE;
        r.follower.setPose(startPose);
        r.follower.startTeleopDrive();

        calibrated = false;
        autoShooterActive = false;
        shootTimer.resetTimer();

        // Put mechanisms in a safe known state
        r.s.stopMotor();
        targetPose = r.getShootTarget();
//        r.s.kickDown();
        r.s.feedZero();

        setLightPos(0.0);
        gamepad1.rumbleBlips(1);
        gamepad2.rumbleBlips(1);
    }

    @Override
    public void loop() {
        r.periodic();
        CommandScheduler.getInstance().run();

        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        drive();
        intake();
        shooterManualLogic();

        Pose robotPose = r.follower.getPose();
        if (robotPose != null && targetPose != null) {
            dist = Math.abs(targetPose.getY() - robotPose.getY());
        }

        // Telemetry
        Pose p = r.follower.getPose();
        telemetry.addData("Alliance", r.a);
        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("AUTO Active?", autoShooterActive);
        telemetry.addData("Calibrated?", calibrated);
        telemetry.addData("Pose", (p == null) ? "null" :
                String.format("(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetry.addData("Distance Used", dist);


        r.s.getTelemetryData(telemetry);

        telemetry.update();
        telemetryM.update();
    }

    // ----------------------------
    // DRIVE + CALIBRATION
    // ----------------------------
    public void drive() {
        // Slow mode toggles
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            setLightPos(0.25);
            gamepad1.rumbleBlips(1);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slowModeActive = false;
            setLightPos(0.0);
            gamepad1.rumbleBlips(2);
        }

        double speedMult = slowModeActive ? adjustSpeed : 1.0;

        r.follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMult,
                -gamepad1.left_stick_x * speedMult,
                -gamepad1.right_stick_x * speedMult,
                true
        );

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            adjustSpeed = Math.min(1.0, adjustSpeed + 0.2);
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            adjustSpeed = Math.max(0.0, adjustSpeed - 0.2);

        // Pose calibrate at top triangle: driver presses Y
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            Pose trianglePose = (r.a == Alliance.RED) ? BLUE_TOP_TRIANGLE_POSE.mirror() : BLUE_TOP_TRIANGLE_POSE;
            r.follower.setPose(trianglePose);
            calibrated = true;

            gamepad1.rumbleBlips(2);
            setLightPos(0.6);
        }
    }

    public void intake() {
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            isIntakeInward = !isIntakeInward;
            isIntakeOutward = false;
            CommandScheduler.getInstance().schedule(isIntakeInward ? r.i.inCommand() : r.i.idleCommand());
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            isIntakeOutward = !isIntakeOutward;
            isIntakeInward = false;
            CommandScheduler.getInstance().schedule(isIntakeOutward ? r.i.outCommand() : r.i.idleCommand());
        }
        if (operatorGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) &&
                operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

            isIntakeInward = false;
            isIntakeOutward = false;
            CommandScheduler.getInstance().schedule(r.i.idleCommand());
        }
    }

    // ----------------------------
    // SHOOTER: MANUAL
    // ----------------------------
    private void shooterManualLogic() {
        // Flywheel presets (tap)
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            r.s.forDistance(0, dist);
            gamepad2.rumbleBlips(1);
            if (r.s.isAtVelocity(r.s.getTarget())){
                r.s.kickUp();
                if (shootTimer.getElapsedTime() > 0.4 && shootTimer.getElapsedTime() < 0.8){
                    r.s.kickDown();
                    r.i.intakeShooter();
                }
                if (shootTimer.getElapsedTime() > 0.8 && shootTimer.getElapsedTime() < 1.2){
                    r.s.kickUp();
                }
                if (shootTimer.getElapsedTime() > 1.2 && shootTimer.getElapsedTime() < 1.6){
                    r.s.kickDown();
                    r.i.intakeShooter();
                }
                if (shootTimer.getElapsedTime() > 1.6 && shootTimer.getElapsedTime() < 2.0){
                    r.s.kickUp();
                }
                if (shootTimer.getElapsedTime() > 2.0){
                    stopAllShooterActions();
                }
            }
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            r.s.intake();
            gamepad2.rumbleBlips(1);
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            r.s.stopMotor();
            r.s.kickDown();
            r.s.feedZero();
            gamepad2.rumbleBlips(2);
        }

        // Kicker control (hold)
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.s.kickUp();
        } else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.s.kickDown();
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            r.s.feedUp();
        } else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            r.s.feedDown();
        }
    }

    private void stopAllShooterActions() {
        autoShooterActive = false;
        r.s.stopMotor();
        r.s.kickDown();
        r.s.feedZero();
        shootTimer.resetTimer();
    }

    // ----------------------------
    // LIGHT
    // ----------------------------
    private void setLightPos(double pos) {
        double clipped = Math.max(0.0, Math.min(1.0, pos));
        if (Math.abs(clipped - currentLightPos) > 0.02) {
            indicatorLight.setPosition(clipped);
            currentLightPos = clipped;
        }
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        stopAllShooterActions();
        r.stop();
        setLightPos(0.0);
    }
}