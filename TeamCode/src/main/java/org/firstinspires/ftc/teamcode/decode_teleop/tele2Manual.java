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
    // Shooter modes (kept, even if unused)
    // ----------------------------
    private enum ShooterMode { AUTO, MANUAL }
    private ShooterMode shooterMode = ShooterMode.MANUAL;

    // AUTO firing state (kept, even if unused)
    private boolean autoShooterActive = false;
    private final Timer shootTimer = new Timer();
    private final Timer indicator = new Timer();

    // Pose-calibration flag
    private boolean calibrated = false;

    // Distance used for shooter calculations
    public double dist = 0.0;

    // ----------------------------
    // TRIPLE SHOT
    // ----------------------------
    private boolean tripleShotEnabled = false;   // START toggles (armed)
    private boolean tripleShotRunning = false;   // true while sequence executes

    private enum TripleState {
        IDLE,
        SPINUP,
        SHOT1,
        RESET1,
        SHOT2,
        RESET2,
        SHOT3,
        CLEANUP
    }

    private TripleState tripleState = TripleState.IDLE;
    private final Timer tripleTimer = new Timer();

    // Tune these timings if needed
    public static double MIN_SPINUP_TIME = 1.2;   // seconds
    public static double KICK_UP_TIME   = 0.25;  // seconds
    public static double FEED_TIME      = 0.40;  // seconds
    public static double GREEN = 0.500;

    Pose targetPose;

    // Put your REAL Pedro field coordinates here for the "top triangle" landmark.
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
            setLightPos(0.277);
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

        // Reset triple-shot state
        tripleShotRunning = false;
        tripleState = TripleState.IDLE;
        tripleTimer.resetTimer();

        // Put mechanisms in a safe known state
        r.s.stopMotor();
        targetPose = r.getShootTarget();
        r.s.feedZero();
        r.s.kickDown();

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

        // Compute dist BEFORE shooter logic so A-press uses fresh dist
        Pose robotPose = r.follower.getPose();
        if (targetPose == null) targetPose = r.getShootTarget();
        if (robotPose != null && targetPose != null) {
            dist = Math.abs(targetPose.getY() - robotPose.getY());
        }

        drive();
        intake();
        shooterManualLogic();

        // Telemetry
        Pose p = r.follower.getPose();
        telemetry.addData("Alliance", r.a);
        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("AUTO Active?", autoShooterActive);
        telemetry.addData("Calibrated?", calibrated);

        telemetry.addData("Triple Enabled", tripleShotEnabled);
        telemetry.addData("Triple Running", tripleShotRunning);
        telemetry.addData("Triple State", tripleState);

        telemetry.addData("Pose", (p == null) ? "null" :
                String.format("(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetry.addData("Distance Used", dist);

        r.s.getTelemetryData(telemetry);

        telemetry.update();
        telemetryM.update();
    }

    // ----------------------------
    // DRIVE + CALIBRATION (UNCHANGED behavior)
    // ----------------------------
    public void drive() {
        // Slow mode toggles
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            setLightPos(0.25);
            if (indicator.getElapsedTime() > 1.0) {
                setLightPos(0.0);
            }
            indicator.resetTimer();
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
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            Pose trianglePose = (r.a == Alliance.RED) ? BLUE_TOP_TRIANGLE_POSE.mirror() : BLUE_TOP_TRIANGLE_POSE;
            r.follower.setPose(trianglePose);
            calibrated = true;

            gamepad1.rumbleBlips(2);
            setLightPos(0.388);
        }
    }

    // ----------------------------
    // INTAKE (UNCHANGED behavior)
    // ----------------------------
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
    // SHOOTER: MANUAL + TRIPLE SHOT OPTION
    // ----------------------------
    private void shooterManualLogic() {

        // Toggle triple-shot enable/disable (armed mode)
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.START)) {
            tripleShotEnabled = !tripleShotEnabled;
            gamepad2.rumbleBlips(tripleShotEnabled ? 2 : 1);

            // If disabled while running, cancel safely
            if (!tripleShotEnabled && tripleShotRunning) {
                cancelTripleShot();
            }
        }

        // A: Spin up always. If triple enabled, start triple sequence.
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            setLightPos(0.500);
            shootTimer.resetTimer();

            // Spin up flywheel + set hood based on distance
            r.s.forDistance(0, dist);
            if (r.s.isAtVelocity(r.s.getTarget())) setLightPos(GREEN);
            gamepad2.rumbleBlips(1);

            if (tripleShotEnabled && !tripleShotRunning) {
                tripleShotRunning = true;
                tripleState = TripleState.SPINUP;
                tripleTimer.resetTimer();
                r.s.kickDown(); // safe start
            }
        }

        // Run triple-shot state machine
        if (tripleShotRunning) {
            double t = tripleTimer.getElapsedTime();

            switch (tripleState) {
                case SPINUP:
                    // Keep commanding distance target during spinup
                    r.s.forDistance(0, dist);

                    // Only proceed when BOTH: time passed and we are at target velocity
                    if (t >= MIN_SPINUP_TIME && r.s.isAtVelocity(r.s.getTarget())) {
                        r.s.kickUp(); // SHOT 1
                        tripleState = TripleState.SHOT1;
                        tripleTimer.resetTimer();
                    }
                    break;

                case SHOT1:
                    if (t >= KICK_UP_TIME) {
                        r.s.kickDown();
                        r.i.intakeShooter(); // feed next
                        tripleState = TripleState.RESET1;
                        tripleTimer.resetTimer();
                    }
                    break;

                case RESET1:
                    if (t >= FEED_TIME) {
                        r.s.kickUp(); // SHOT 2
                        tripleState = TripleState.SHOT2;
                        tripleTimer.resetTimer();
                    }
                    break;

                case SHOT2:
                    if (t >= KICK_UP_TIME) {
                        r.s.kickDown();
                        r.i.intakeShooter(); // feed next
                        tripleState = TripleState.RESET2;
                        tripleTimer.resetTimer();
                    }
                    break;

                case RESET2:
                    if (t >= FEED_TIME) {
                        r.s.kickUp(); // SHOT 3
                        tripleState = TripleState.SHOT3;
                        tripleTimer.resetTimer();
                    }
                    break;

                case SHOT3:
                    if (t >= KICK_UP_TIME) {
                        tripleState = TripleState.CLEANUP;
                        tripleTimer.resetTimer();
                    }
                    break;

                case CLEANUP:
                    stopAllShooterActions();
                    tripleShotRunning = false;
                    tripleState = TripleState.IDLE;
                    setLightPos(0.0);
                    break;

                default:
                    tripleShotRunning = false;
                    tripleState = TripleState.IDLE;
                    break;
            }
        }

        // Other Manual Controls (unchanged)
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            r.i.intakeShooter();
            gamepad2.rumbleBlips(1);
        }

        // Cancel (X) always cancels triple-shot + stops shooter
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            cancelTripleShot();
            setLightPos(0.0);
        }

        // Manual Kicker/Feeder overrides (unchanged)
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            r.s.kickUp();
            r.i.spinIdle();
        }
        else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            r.s.kickDown();
            r.i.intakeShooter();
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) r.s.feedUp();
        else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) r.s.feedDown();
    }

    private void cancelTripleShot() {
        stopAllShooterActions();
        tripleShotRunning = false;
        tripleState = TripleState.IDLE;
        setLightPos(0.0);
    }

    private void stopAllShooterActions() {
        autoShooterActive = false;
        r.s.stopMotor();
        r.s.kickDown();
        r.s.feedZero();
        setLightPos(0.0);
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
        cancelTripleShot();
        r.stop();
        setLightPos(0.0);
    }
}