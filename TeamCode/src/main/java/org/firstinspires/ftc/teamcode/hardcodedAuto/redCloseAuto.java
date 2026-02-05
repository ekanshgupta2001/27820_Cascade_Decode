package org.firstinspires.ftc.teamcode.hardcodedAuto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.decode_teleop.tele2Manual;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class redCloseAuto extends LinearOpMode {
    private Follower follower;
    NonVisionRobot r;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
    private Shooter s;
    private Intake i;
    private DcMotor lfMotor = null;
    private DcMotor lrMotor = null;
    private DcMotor rfMotor = null;
    private DcMotor rrMotor = null;
    private DcMotor scoreMotor = null;
    private DcMotor intakeMotor = null;
    private Servo adjustHood = null;
    private TelemetryManager telemetryM;
    private DcMotorEx parallelEncoder;
    private DcMotorEx perpendicularEncoder;
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
    public static double AUTO_SPINUP_MIN_TIME = 1.0;  // Minimum time to spin up
    public static double AUTO_KICK_UP_TIME = 0.25;    // How long kicker stays up
    public static double AUTO_RESET_TIME = 0.4;
    private double currentPower = 0;
    private double adjustSpeed = 0.5;
    private double intakeDirection = 0.0;
    private double lastTime;
    private int lastPosition = 0;
    private double velocity;
    private double timeCheck;
    private boolean slowModeActive = false;
    @Override
    public void runOpMode() {

        s = new Shooter(hardwareMap, i);
        i = new Intake(hardwareMap);

        lfMotor = hardwareMap.get(DcMotor.class, "FL");
        lrMotor = hardwareMap.get(DcMotor.class, "BL");
        rfMotor = hardwareMap.get(DcMotor.class, "FR");
        rrMotor = hardwareMap.get(DcMotor.class, "BR");

        intakeMotor = hardwareMap.get(DcMotor.class, "IM");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        scoreMotor = hardwareMap.get(DcMotor.class, "SM");
        scoreMotor.setDirection(DcMotor.Direction.FORWARD);

        adjustHood = hardwareMap.get(Servo.class, "AH");
        adjustHood.setDirection(Servo.Direction.FORWARD);

        lfMotor.setDirection(DcMotor.Direction.FORWARD);
        lrMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rrMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();



        Auto();
        sleep(100000);
        telemetry.addData("Auto phase", "Done");
        telemetry.update();
    }
    public void Auto() {
        new Shoot(r);
        move(1300, -0.5);
        if (autoTimer.getElapsedTime() > 2500){
            rotate(400, 0.8);
            autoTimer.resetTimer();
            if (autoTimer.getElapsedTime() > 300){
                move(1000, 0.8);
                intake(-1.0, 2000);
                autoTimer.resetTimer();
                if (autoTimer.getElapsedTime() > 2000){
                    move(1000, -0.8);
                    autoTimer.resetTimer();
                    if (autoTimer.getElapsedTime() > 1000){
                        rotate(400, -0.8);
                        new Shoot(r);
                        autoTimer.resetTimer();
                        if (autoTimer.getElapsedTime() > 3000){
                            moveSideways(0.5, 500);
                        }
                    }
                }
            }
        }

    }

    public void move(long movement, double speed) {
        telemetry.addData("Current Status", "Moving");
        telemetry.update();
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        frontLeftPower = speed;
        backLeftPower = speed;
        frontRightPower = speed;
        backRightPower = speed;

        lfMotor.setPower(frontLeftPower);
        lrMotor.setPower(backLeftPower);
        rfMotor.setPower(frontRightPower);
        rrMotor.setPower(backRightPower);

        sleep(movement);
        lfMotor.setPower(0);
        lrMotor.setPower(0);
        rfMotor.setPower(0);
        rrMotor.setPower(0);
    }

    public void moveSideways(double speed, long time){
        lfMotor.setPower(-speed);
        rfMotor.setPower(speed);
        lrMotor.setPower(speed);
        rrMotor.setPower(-speed);

        sleep(time);

        lfMotor.setPower(0.0);
        rfMotor.setPower(0.0);
        lrMotor.setPower(0.0);
        rrMotor.setPower(0.0);

    }

    public void intake(double power, long time){
        intakeMotor.setPower(power);

        sleep(time);
        intakeMotor.setPower(0.0);
    }

    public void rotate(long degrees, double power) {
        telemetry.addData("Current Status", "Rotate");
        telemetry.update();

        lfMotor.setPower(-power);
        rfMotor.setPower(-power);
        lrMotor.setPower(power);
        rrMotor.setPower(power);

        sleep(degrees);

        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lrMotor.setPower(0);
        rrMotor.setPower(0);

    }

    public void hood(double angle){
        adjustHood.setPosition(angle);
    }




}
