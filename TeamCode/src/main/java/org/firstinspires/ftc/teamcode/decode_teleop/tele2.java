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
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@TeleOp
@Config
public class tele2 extends OpMode {
    Robot r;
    private Servo indicatorLight;
    private double currentLightPos = 0.0;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    MultipleTelemetry multipleTelemetry;
    private final int RED_SCORE_ZONE_ID = 24;
    private final int BLUE_SCORE_ZONE_ID = 20;
    private TelemetryManager telemetryM;
    private boolean slowModeActive = false;
    private double adjustSpeed = 0.25;
    private boolean isIntakeInward = false;
    private boolean isIntakeOutward = false;
    private boolean isShooterActive = false;
    private final Timer shoot = new Timer();
    private final Timer kick = new Timer();
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
            setLightPosition(0.87);
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.w.setTargetTagID(RED_SCORE_ZONE_ID);
            r.setAlliance(Alliance.RED);
            setLightPosition(0.61);
        }
    }

    @Override
    public void start() {
        if (r.w.isTagVisible(r.w.getTargetTagID())) {
            Pose trueFieldPose = r.w.getRobotPoseFieldSpace(r.w.getTargetTagID());
            if (trueFieldPose != null) {
                r.follower.setPose(trueFieldPose);
                gamepad1.rumble(500);
            }
        }

        setLightPosition(-0.95);
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

        hood();
        intake();
        drive();

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            isShooterActive = true;
            shoot.resetTimer();
            gamepad2.rumbleBlips(1);
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            r.s.stopMotor();
            isShooterActive = false;
            gamepad2.rumbleBlips(2);
        }

        r.w.displayTagTelemetry(r.w.getTargetTag());
        r.s.getTelemetryData(telemetry);
        r.i.getTelemetryData(telemetry);
        telemetryM.debug("Position: ", r.follower.getPose());
        telemetry.addData("Distance Used", dist);
        telemetry.update();
        telemetryM.update();
    }

    public void drive() {
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slowModeActive = true;
            setLightPosition(-0.29);
            gamepad1.rumbleBlips(1);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slowModeActive = false;
            setLightPosition(-0.83);
            gamepad1.rumbleBlips(2);
        }

        double speedMult = slowModeActive ? adjustSpeed : 1.0;
        r.follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMult,
                -gamepad1.left_stick_x * speedMult,
                -gamepad1.right_stick_x * speedMult,
                false
        );

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) adjustSpeed = Math.min(1.0, adjustSpeed + 0.2);
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) adjustSpeed = Math.max(0.0, adjustSpeed - 0.2);
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

    public void hood() {
        if (r.w.isTagVisible(r.w.getTargetTagID())) {
            dist = r.w.getRangeToTag();
        } else {
            dist = r.getShootTarget().distanceFrom(r.follower.getPose());
        }

        if (isShooterActive) {
            CommandScheduler.getInstance().schedule(r.i.idleCommand());
            r.s.forDistance(dist);

            if (shoot.getElapsedTime() > 0.8) {
                r.s.kickUp();
                if (kick.getElapsedTime() > 0.5) {
                    r.s.kickDown();
                    r.i.spinIn();
                }
                shoot.resetTimer();
                kick.resetTimer();
            }
            if (shoot.getElapsedTime() > 0.4) {
                r.s.kickUp();
                if (kick.getElapsedTime() > 0.5) {
                    r.s.kickDown();
                    r.i.spinIn();
                }
                shoot.resetTimer();
                kick.resetTimer();
            }
            if (shoot.getElapsedTime() > 0.4) {
                r.s.kickUp();
                if (kick.getElapsedTime() > 0.5) {
                    r.s.kickDown();
                    r.i.spinIn();
                }
            }
        }
    }

    private void setLightPosition(double pos) {
        if (Math.abs(pos - currentLightPos) > 0.05) {
            indicatorLight.setPosition(pos);
            currentLightPos = pos;
        }
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
