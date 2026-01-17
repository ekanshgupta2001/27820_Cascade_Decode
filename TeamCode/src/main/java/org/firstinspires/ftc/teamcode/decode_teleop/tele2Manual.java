package org.firstinspires.ftc.teamcode.decode_teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@TeleOp(name = "tele2Manual_2026")
@Config
public class tele2Manual extends OpMode {
    NonVisionRobot r;
    private Servo indicatorLight;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private boolean slowModeActive = false;
    private double adjustSpeed = 0.25;
    private final Timer indicator = new Timer();

    // Triple Shot State Machine
    private boolean tripleShotRunning = false;
    private enum TripleState { IDLE, SPINUP, SHOT1, RESET1, SHOT2, RESET2, SHOT3, CLEANUP }
    private TripleState tripleState = TripleState.IDLE;
    private final Timer tripleTimer = new Timer();
    public double dist = 0.0;

    @Override
    public void init() {
        r = new NonVisionRobot(hardwareMap, telemetry, Alliance.BLUE);
        indicatorLight = hardwareMap.get(Servo.class, "IL");
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        setLightPos(0.0);
    }

    @Override
    public void start() {
        r.follower.startTeleopDrive();
        tripleShotRunning = false;
        tripleState = TripleState.IDLE;
        r.s.stopMotor();
        setLightPos(0.0);
    }

    @Override
    public void loop() {
        r.periodic();
        CommandScheduler.getInstance().run();
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        // Calculate distance for the 2026 goal
        Pose robotPose = r.follower.getPose();
        if (robotPose != null && r.getShootTarget() != null) {
            dist = Math.abs(r.getShootTarget().getY() - robotPose.getY());
        }

        drive();
        intake();
        shooterManualLogic();

        telemetry.addData("Triple State", tripleState);
        telemetry.addData("Goal Distance", dist);
        r.s.getTelemetryData(telemetry);
        telemetry.update();
    }

    private void setLightPos(double pos) {
        indicatorLight.setPosition(pos);
    }

    public void drive() {
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            indicator.resetTimer();
            setLightPos(0.25);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) slowModeActive = false;
        if (slowModeActive && indicator.getElapsedTime() > 1.0) setLightPos(0.0);

        double speedMult = slowModeActive ? adjustSpeed : 1.0;
        r.follower.setTeleOpDrive(-gamepad1.left_stick_y * speedMult, -gamepad1.left_stick_x * speedMult, -gamepad1.right_stick_x * speedMult, true);
    }

    public void intake() {
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            CommandScheduler.getInstance().schedule(r.i.inCommand());
        } else if (operatorGamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            CommandScheduler.getInstance().schedule(r.i.idleCommand());
        }
    }

    private void shooterManualLogic() {
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A) && !tripleShotRunning) {
            tripleShotRunning = true;
            tripleState = TripleState.SPINUP;
            tripleTimer.resetTimer();
        }

        switch (tripleState) {
            case SPINUP:
                r.s.forDistance(0, dist);
                if (tripleTimer.getElapsedTime() > 0.8) { tripleState = TripleState.SHOT1; tripleTimer.resetTimer(); }
                break;
            case SHOT1: r.s.kickUp();
                if (tripleTimer.getElapsedTime() > 0.2) { tripleState = TripleState.RESET1; tripleTimer.resetTimer(); }
                break;
            case RESET1: r.s.kickDown();
                if (tripleTimer.getElapsedTime() > 0.4) { tripleState = TripleState.SHOT2; tripleTimer.resetTimer(); }
                break;
            case SHOT2: r.s.kickUp();
                if (tripleTimer.getElapsedTime() > 0.2) { tripleState = TripleState.RESET2; tripleTimer.resetTimer(); }
                break;
            case RESET2: r.s.kickDown();
                if (tripleTimer.getElapsedTime() > 0.4) { tripleState = TripleState.SHOT3; tripleTimer.resetTimer(); }
                break;
            case SHOT3: r.s.kickUp();
                if (tripleTimer.getElapsedTime() > 0.2) { tripleState = TripleState.CLEANUP; tripleTimer.resetTimer(); }
                break;
            case CLEANUP: r.s.kickDown(); r.s.stopMotor(); tripleShotRunning = false; tripleState = TripleState.IDLE;
                break;
            default: break;
        }
    }
}
