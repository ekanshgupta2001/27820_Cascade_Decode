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
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.NonVisionRobot;

@TeleOp(name = "tele2NoVision")
@Config
public class tele2NonVision extends OpMode {

    NonVisionRobot r;

    // goBILDA PWM light should be controlled like a SERVO (0..1 position).
    // Configure it as a Servo device in the Robot Config.
    private Servo indicatorLight;
    private double currentLightPos = 0.0;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    MultipleTelemetry multipleTelemetry;
    private TelemetryManager telemetryM;

    private boolean slowModeActive = false;
    private double adjustSpeed = 0.25;

    private boolean isIntakeInward = false;
    private boolean isIntakeOutward = false;
    private boolean isShooterActive = false;

    private final Timer shoot = new Timer();

    public double dist;

    // ----------------------------
    // IMPORTANT: FILL THESE IN
    // ----------------------------
    // These should be the real field pose (x, y, headingRadians) for the "top triangle"
    // that your drivers can reliably hit. Heading matters: set it to how you want the
    // robot to be oriented after calibration.
    private static final Pose BLUE_TOP_TRIANGLE_POSE = new Pose(
            0,   // TODO: replace with real X
            0,   // TODO: replace with real Y
            0    // TODO: replace with real heading (radians)
    );

    // Set a real start pose for TeleOp so Pedro has something valid immediately.
    private static final Pose BLUE_START_POSE = new Pose(
            45, 100, 0 // TODO: replace with your real start pose
    );

    private boolean calibrated = false;

    @Override
    public void init() {
        // Build the NO-VISION robot. Default alliance BLUE; you can switch in init_loop().
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);

        // Map your light as a Servo (not CRServo).
        // Make sure your Robot Config device name matches "IL".
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

        // Operator picks alliance during init (no webcam/tag selection anymore).
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.setAlliance(Alliance.BLUE);
            setLightPos(0.6); // blue vibe (brightness only)
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.setAlliance(Alliance.RED);
            setLightPos(0.6); // red vibe (brightness only)
        }

        telemetry.addData("Alliance", r.a);
        telemetry.addData("Calibrated?", calibrated);
        telemetry.update();
    }

    @Override
    public void start() {
        // Set a valid starting pose (mirrored for RED).
        Pose startPose = (r.a == Alliance.RED) ? BLUE_START_POSE.mirror() : BLUE_START_POSE;
        r.follower.setPose(startPose);

        // Start teleop drive mode for follower.
        r.follower.startTeleopDrive();

        calibrated = false;
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
        shooterLogic();

        // Telemetry
        Pose p = r.follower.getPose();
        telemetry.addData("Alliance", r.a);
        telemetry.addData("Calibrated?", calibrated);
        telemetry.addData("Pose", (p == null) ? "null" : String.format("(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetry.addData("Distance Used", dist);

        r.s.getTelemetryData(telemetry);

        telemetry.update();
        telemetryM.update();
    }

    public void drive() {
        // Toggle slow mode for precision.
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

        // Normal Pedro teleop drive.
        r.follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMult,
                -gamepad1.left_stick_x * speedMult,
                -gamepad1.right_stick_x * speedMult,
                false
        );

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            adjustSpeed = Math.min(1.0, adjustSpeed + 0.2);
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            adjustSpeed = Math.max(0.0, adjustSpeed - 0.2);

        // ----------------------------
        // MANUAL POSE CALIBRATION
        // ----------------------------
        // Driver drives to top triangle, presses Y, we "snap" Pedro's pose to the known field pose.
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            Pose trianglePose = getTopTrianglePoseForAlliance();
            r.follower.setPose(trianglePose);
            calibrated = true;

            // Feedback
            gamepad1.rumbleBlips(2);
            setLightPos(0.6);
        }
    }

    private Pose getTopTrianglePoseForAlliance() {
        return (r.a == Alliance.RED) ? BLUE_TOP_TRIANGLE_POSE.mirror() : BLUE_TOP_TRIANGLE_POSE;
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
    }

    public void shooterLogic() {
        // No webcam: distance is purely geometry-based from your localization pose.
        Pose robotPose = r.follower.getPose();
        if (robotPose == null) return;

        dist = r.getShootTarget().distanceFrom(robotPose);

        // Start shooting on A.
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            isShooterActive = true;
            shoot.resetTimer();
            gamepad2.rumbleBlips(1);
        }

        // Stop shooter on X.
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            r.s.stopMotor();
            r.s.kickDown();
            isShooterActive = false;
            gamepad2.rumbleBlips(2);
        }

        if (!isShooterActive) return;

        r.s.forDistance(dist);

        double t = shoot.getElapsedTime();

        if (t < 0.25) {
            r.s.kickUp();
            r.i.spinIdle();
        } else if (t < 0.45) {
            r.s.kickDown();
            r.i.spinIdle();
        } else {
            r.i.spinIn();
            shoot.resetTimer();
        }
    }

    private void setLightPos(double pos) {
        // Servo position must be 0..1.
        double clipped = Math.max(0.0, Math.min(1.0, pos));
        if (Math.abs(clipped - currentLightPos) > 0.02) {
            indicatorLight.setPosition(clipped);
            currentLightPos = clipped;
        }
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        r.stop();
        setLightPos(0.0);
    }
}