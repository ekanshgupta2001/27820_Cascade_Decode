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
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Config
public class tele2 extends OpMode {
    Robot r;

    // This is the goBILDA headlight (PWM controlled), so it should be CRServo-style power.
    private CRServo indicatorLight;
    private double currentLightPower = 0.0;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    MultipleTelemetry multipleTelemetry;
    private TelemetryManager telemetryM;

    private final int RED_SCORE_ZONE_ID = 24;
    private final int BLUE_SCORE_ZONE_ID = 20;

    private boolean slowModeActive = false;
    private double adjustSpeed = 0.25;

    private boolean isIntakeInward = false;
    private boolean isIntakeOutward = false;
    private boolean isShooterActive = false;

    private final Timer shoot = new Timer();

    public double dist;

    @Override
    public void init() {
        // This builds our robot and defaults alliance to BLUE (we can change it in init loop).
        r = new Robot(hardwareMap, telemetry, Alliance.BLUE);

        // This maps the headlight PWM device as a CRServo (power is -1..1).
        indicatorLight = hardwareMap.get(CRServo.class, "IL");

        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void init_loop() {
        // This reads button edge events during init so alliance/tag selection is clean.
        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.w.setTargetTagID(BLUE_SCORE_ZONE_ID);
            r.setAlliance(Alliance.BLUE);
            setLightPower(0.6); // “blue vibe”
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.w.setTargetTagID(RED_SCORE_ZONE_ID);
            r.setAlliance(Alliance.RED);
            setLightPower(-0.6); // “red vibe”
        }
    }

    @Override
    public void start() {
        // This sets a valid starting pose no matter what, so Pedro never crashes.
        Pose startPose = new Pose(12, 12, 0); // TODO: replace with your real start pose
        if (r.w.isTagVisible(r.w.getTargetTagID())) {
            Pose trueFieldPose = r.w.getRobotPoseFieldSpace(r.w.getTargetTagID());
            if (trueFieldPose != null) startPose = trueFieldPose;
        }
        r.follower.setPose(startPose);

        // This starts teleop drive mode for follower.
        r.follower.startTeleopDrive();

        setLightPower(0.0);
        gamepad1.rumbleBlips(1);
        gamepad2.rumbleBlips(1);
    }

    @Override
    public void loop() {
        // This keeps subsystems + follower updated each loop.
        r.periodic();

        // This runs the command scheduler so commands actually execute.
        CommandScheduler.getInstance().run();

        driverGamepad.readButtons();
        operatorGamepad.readButtons();

        drive();
        intake();
        shooterLogic();

        r.w.displayTagTelemetry(r.w.getTargetTag());
        r.s.getTelemetryData(telemetry);
        telemetry.addData("Distance Used", dist);

        telemetry.update();
        telemetryM.update();
    }

    public void drive() {
        // This toggles slow mode so driver can be more precise.
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            setLightPower(0.25);
            gamepad1.rumbleBlips(1);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slowModeActive = false;
            setLightPower(0.0);
            gamepad1.rumbleBlips(2);
        }

        double speedMult = slowModeActive ? adjustSpeed : 1.0;

        // This sends joystick inputs into Pedro teleop drive.
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
    }

    public void intake() {
        // This toggles intake in/out using commands without constantly rescheduling.
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
        // This gets distance either from AprilTag (if visible) or from geometry as a fallback.
        if (r.w.isTagVisible(r.w.getTargetTagID())) dist = r.w.getRangeToTag();
        else dist = r.getShootTarget().distanceFrom(r.follower.getPose());

        // This starts shooting when A is pressed.
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            isShooterActive = true;
            shoot.resetTimer();
            gamepad2.rumbleBlips(1);
        }

        // This stops shooter when X is pressed.
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            r.s.stopMotor();
            r.s.kickDown();
            isShooterActive = false;
            gamepad2.rumbleBlips(2);
        }

        if (!isShooterActive) return;

        // This keeps shooter spinning to a distance-based target.
        r.s.forDistance(dist);

        // This is a simple, repeatable kick cycle so it doesn’t “timer reset spam.”
        double t = shoot.getElapsedTime();

        if (t < 0.25) {
            r.s.kickUp();
            r.i.spinIdle();
        } else if (t < 0.45) {
            r.s.kickDown();
            r.i.spinIdle();
        } else {
            // Cycle complete: push next artifact a bit, then restart.
            r.i.spinIn();
            shoot.resetTimer();
        }
    }

    private void setLightPower(double power) {
        // This clamps light power so it stays within the valid PWM range.
        double clipped = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(clipped - currentLightPower) > 0.05) {
            indicatorLight.setPower(clipped);
            currentLightPower = clipped;
        }
    }

    @Override
    public void stop() {
        // This clears command scheduler so old commands don’t haunt the next opmode.
        CommandScheduler.getInstance().reset();
    }
}