package org.firstinspires.ftc.teamcode.decode_teleop;

import static org.firstinspires.ftc.teamcode.commands.Shoot.MIN_SPINUP_TIME;

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

    // Shooter modes
    private enum ShooterMode { AUTO, MANUAL }
    private ShooterMode shooterMode = ShooterMode.MANUAL;

    // AUTO mode state machine
    private enum AutoShootState {
        IDLE,           // Not shooting
        SPINNING_UP,    // Flywheel spinning to target velocity
        READY,          // At velocity, waiting for trigger
        SHOOTING,       // Kicker up, releasing sample
        RESETTING,      // Kicker down, feeding next sample
        COMPLETE        // All 3 shots done
    }
    private AutoShootState autoState = AutoShootState.IDLE;
    private final Timer autoTimer = new Timer();
    private final Timer intakeTimer = new Timer();

    private int shotsRemaining = 0;  // Track how many shots left in sequence

    // Light indicator timer
    private final Timer indicator = new Timer();

    private boolean calibrated = false;
    public double dist = 0.0;

    // Timings for AUTO mode (tunable via dashboard)
    public static double AUTO_SPINUP_MIN_TIME = 1.0;  // Minimum time to spin up
    public static double AUTO_KICK_UP_TIME = 0.25;    // How long kicker stays up
    public static double AUTO_RESET_TIME = 0.4;       // How long to wait before next shot
    public static double LIGHT_READY_COLOR = 0.500;   // Green when ready
    public static double LIGHT_SHOOTING_COLOR = 0.388; // Different color while shooting
    public static int MAX_SHOTS = 3;                   // Total samples to shoot
    private boolean fieldCentricMode = true;

    Pose targetPose;

    private static final Pose BLUE_TOP_TRIANGLE_POSE = new Pose(72, 72, 0);
    private static final Pose BLUE_START_POSE = new Pose(12, 12, 0);

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
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startPose = (r.a == Alliance.RED) ? BLUE_START_POSE.mirror() : BLUE_START_POSE;
        r.follower.setPose(startPose);
        r.follower.startTeleopDrive();

        calibrated = false;
        autoState = AutoShootState.IDLE;
        autoTimer.resetTimer();

        r.s.setTarget(250);
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

        // Calculate distance for shooter targeting
        Pose robotPose = r.follower.getPose();
        if (targetPose == null) targetPose = r.getShootTarget();
        if (robotPose != null && targetPose != null) {
            dist = Math.abs(targetPose.getY() - robotPose.getY());
        }

        drive();
        intake();

        // Switch between AUTO and MANUAL shooter modes
        if (shooterMode == ShooterMode.AUTO) {
            shooterAutoLogic();
        } else {
            shooterManualLogic();
        }

        // Telemetry
        Pose p = r.follower.getPose();
        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("Alliance", r.a);
        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Auto State", autoState);
        telemetry.addData("Shots Remaining", shotsRemaining);
        telemetry.addData("Calibrated?", calibrated);
        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("Pose", (p == null) ? "null" :
                String.format("(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetry.addData("Distance Y", "%.1f", dist);

        r.s.getTelemetryData(telemetry);
        telemetry.update();
        telemetryM.update();
    }

    // ═══════════════════════════════════════════════════════════
    // DRIVE + CALIBRATION
    // ═══════════════════════════════════════════════════════════
    public void drive() {
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            setLightPos(0.25);
            indicator.resetTimer();
            gamepad1.rumbleBlips(1);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slowModeActive = false;
            setLightPos(0.0);
            gamepad1.rumbleBlips(2);
        }

        // Clear indicator light after 1 second
        if (indicator.getElapsedTime() > 1.0 && slowModeActive) {
            setLightPos(0.0);
        }


        double speedMult = slowModeActive ? adjustSpeed : 1.0;

        r.follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMult,
                -gamepad1.left_stick_x * speedMult,
                -gamepad1.right_stick_x * speedMult,
                fieldCentricMode
        );

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.START)){
            fieldCentricMode = !fieldCentricMode;
            gamepad1.rumbleBlips(3);
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            adjustSpeed = Math.min(1.0, adjustSpeed + 0.2);
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            adjustSpeed = Math.max(0.0, adjustSpeed - 0.2);

        // Calibrate pose at top triangle
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            Pose trianglePose = (r.a == Alliance.RED)
                    ? BLUE_TOP_TRIANGLE_POSE.mirror()
                    : BLUE_TOP_TRIANGLE_POSE;
            r.follower.setPose(trianglePose);
            calibrated = true;
            gamepad1.rumbleBlips(2);
            setLightPos(0.388);
        }
    }

    // ═══════════════════════════════════════════════════════════
    // INTAKE
    // ═══════════════════════════════════════════════════════════
    public void intake() {
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            isIntakeInward = !isIntakeInward;
            isIntakeOutward = false;
            CommandScheduler.getInstance().schedule(
                    isIntakeInward ? r.i.inCommand() : r.i.idleCommand()
            );
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            isIntakeOutward = !isIntakeOutward;
            isIntakeInward = false;
            CommandScheduler.getInstance().schedule(
                    isIntakeOutward ? r.i.outCommand() : r.i.idleCommand()
            );
        }

        // Both bumpers = stop
        if (operatorGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) &&
                operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            isIntakeInward = false;
            isIntakeOutward = false;
            CommandScheduler.getInstance().schedule(r.i.idleCommand());
        }
    }

    // ═══════════════════════════════════════════════════════════
    // SHOOTER: AUTO MODE - Shoots 3 samples automatically
    // ═══════════════════════════════════════════════════════════
    private void shooterAutoLogic() {
        // START toggles between AUTO and MANUAL
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.START)) {
            shooterMode = ShooterMode.MANUAL;
            autoState = AutoShootState.IDLE;
            shotsRemaining = 0;
            stopAllShooterActions();
            gamepad2.rumbleBlips(1);
            return;
        }

        // X button: EMERGENCY STOP - works in ANY state
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            stopAllShooterActions();
            autoState = AutoShootState.IDLE;
            shotsRemaining = 0;
            setLightPos(0.0);
            gamepad2.rumble(500);  // Long rumble = emergency stopped
            return;
        }

        // Run AUTO state machine
        double elapsed = autoTimer.getElapsedTime();

        switch (autoState) {
            case IDLE:
                // A button: Start 3-shot sequence
                if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    r.s.forDistance(dist);  // Set target based on distance
                    r.s.kickDown();            // Ensure kicker is down
                    r.i.spinIdle();
                    shotsRemaining = MAX_SHOTS;
                    autoState = AutoShootState.SPINNING_UP;
                    autoTimer.resetTimer();
                    setLightPos(0.25);  // Yellow/orange = spinning up
                    gamepad2.rumbleBlips(1);
                }

                // B button: Manual feed assist even in IDLE
                if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    r.i.intakeShooter();
                }
                break;

            // In shooterAutoLogic() SPINNING_UP state:
            case SPINNING_UP:
                r.s.forDistance(dist);

                // Add feedback so driver knows to wait
                if (elapsed < 2.0) {
                    gamepad2.rumble(100);  // Constant rumble = still spinning up
                    setLightPos(0.25);     // Yellow = not ready
                }

                if (elapsed >= MIN_SPINUP_TIME && r.s.isAtVelocity(r.s.getTarget())) {
                    autoState = AutoShootState.READY;
                    setLightPos(LIGHT_READY_COLOR);  // GREEN = READY!
                    gamepad2.rumble(500);  // Long rumble = READY NOW!
                }
                break;

            case READY:
                // Maintain velocity
                r.s.forDistance(dist);

                // Automatically start shooting after brief delay (or immediately)
                // You can add a delay here if you want drivers to have a moment
                if (elapsed >= 0.3) {  // 0.3 second delay before auto-shooting
                    r.s.kickUp();
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    autoState = AutoShootState.SHOOTING;
                    autoTimer.resetTimer();
                    setLightPos(LIGHT_SHOOTING_COLOR);
                }

                // B button: Feed assist while waiting
                if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    r.i.intakeShooter();
                }
                break;

            case SHOOTING:
                // Wait for kick to complete
                if (elapsed >= AUTO_KICK_UP_TIME) {
                    r.s.kickDown();
                    shotsRemaining--;
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    // Check if we're done or need to continue
                    if (shotsRemaining <= 0) {
                        autoState = AutoShootState.COMPLETE;
                        autoTimer.resetTimer();
                    } else {
                        r.i.intakeShooter();  // Feed next sample
                        try {
                            Thread.sleep(700);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        autoState = AutoShootState.RESETTING;
                        autoTimer.resetTimer();
                    }
                }
                break;

            case RESETTING:
                // Keep flywheel at speed
                r.s.forDistance(dist);

                // Wait for feed to complete, then shoot again
                if (elapsed >= AUTO_RESET_TIME) {
                    r.s.kickUp();  // Shoot next sample immediately
                    autoState = AutoShootState.SHOOTING;
                    autoTimer.resetTimer();
                    setLightPos(LIGHT_SHOOTING_COLOR);
                }
                break;

            case COMPLETE:
                // All 3 shots done - stop everything after brief delay
                if (elapsed >= 0.5) {
                    stopAllShooterActions();
                    autoState = AutoShootState.IDLE;
                    shotsRemaining = 0;
                    gamepad2.rumbleBlips(3);  // 3 rumbles = sequence complete
                }
                break;
        }


        if (operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            r.i.shooterinCommand();
        }
    }

    // ═══════════════════════════════════════════════════════════
    // SHOOTER: MANUAL MODE
    // ═══════════════════════════════════════════════════════════
    private void shooterManualLogic() {
        // START toggles to AUTO mode
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.START)) {
            shooterMode = ShooterMode.AUTO;
            autoState = AutoShootState.IDLE;
            stopAllShooterActions();
            gamepad2.rumbleBlips(2);
            return;
        }

        // A: Spin up for distance
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            r.s.forDistance(dist);
            setLightPos(0.25);  // Yellow/orange
            gamepad2.rumbleBlips(1);
        }

        // Update light to green when at velocity (check continuously)
        if (r.s.getTarget() > 0 && r.s.isAtVelocity(r.s.getTarget())) {
            setLightPos(LIGHT_READY_COLOR);  // Green
        }

        // B: Feed intake into shooter
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            r.i.intakeShooter();
            gamepad2.rumbleBlips(1);
        }

        // X: Stop shooter
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            stopAllShooterActions();
        }

        // D-pad UP: Kick up
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.s.kickUp();
            r.i.spinIdle();
        }

        // D-pad DOWN: Kick down + feed
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.s.kickDown();
            r.i.intakeShooter();
            if (intakeTimer.getElapsedTimeSeconds() > 1.0){
                r.i.spinIdle();
            }
        }

        // D-pad LEFT/RIGHT: Adjust hood
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            r.s.feedUp();
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            r.s.feedDown();
        }
    }

    // ═══════════════════════════════════════════════════════════
    // HELPER FUNCTIONS
    // ═══════════════════════════════════════════════════════════
    private void stopAllShooterActions() {
        r.s.setTarget(250);
        r.s.kickDown();
        r.s.feedZero();
        r.i.spinIdle();
        setLightPos(0.0);
        autoTimer.resetTimer();
    }

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