package org.firstinspires.ftc.teamcode.decode_teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class cascadeTeleop extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
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
    private double currentPower = 0;
    private double adjustSpeed = 0.5;
    private double intakeDirection = 0.0;
    private double lastTime;
    private int lastPosition = 0;
    private double velocity;
    private double timeCheck;
    private boolean slowModeActive = false;
    private double currentHoodPosition = 0.0;
//    public adjusthood hood;
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "IM");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        scoreMotor = hardwareMap.get(DcMotor.class, "SM");
        scoreMotor.setDirection(DcMotor.Direction.FORWARD);

        adjustHood = hardwareMap.get(Servo.class, "AH");
        adjustHood.setDirection(Servo.Direction.FORWARD);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        lfMotor = hardwareMap.get(DcMotor.class, "FL");
        lrMotor = hardwareMap.get(DcMotor.class, "BL");
        rfMotor = hardwareMap.get(DcMotor.class, "FR");
        rrMotor = hardwareMap.get(DcMotor.class, "BR");

        lfMotor.setDirection(DcMotor.Direction.FORWARD);
        lrMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rrMotor.setDirection(DcMotor.Direction.REVERSE);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        lastPosition = scoreMotor.getCurrentPosition();
        runtime.reset();
        lastTime = runtime.seconds();
        timeCheck = runtime.seconds();

        telemetry.addLine("Auto complete");
        telemetry.update();

    }
//
//    @Override
//    public void start(){
//        Pose startPose = new Pose(56, 8, 0);
//        follower.setStartingPose(startPose);
//        runtime.reset();
//
//    }

    @Override
    public void loop() {
        movement();
        intake();
        hood();
        scoreArt();

        double currentTime = runtime.seconds();
        int currentPosition = scoreMotor.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaPosition = currentPosition - lastPosition;

        velocity = deltaPosition / deltaTime;

        lastTime = currentTime;
        lastPosition = currentPosition;

        telemetry.addData("Motor Velocity", velocity);

        telemetry.addData("Front Left Power", lfMotor.getPower());
        telemetry.addData("Front Right Power", rfMotor.getPower());
        telemetry.addData("Back Left Power", lrMotor.getPower());
        telemetry.addData("Back Right Power", rrMotor.getPower());
        telemetry.addData("Score Motor Power", scoreMotor.getPower());
        telemetry.addData("Intake Motor Power", intakeMotor.getPower());
//        telemetry.addData("Perpendicular Encoders", perpendicularEncoder);
//        telemetry.addData("Parallel Encoders", parallelEncoder);
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.update();
    }

    public void  movement(){
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x; // Left/right
        double rotation = gamepad1.right_stick_x; // Rotation

        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        lfMotor.setPower(frontLeftPower);
        rfMotor.setPower(frontRightPower);
        lrMotor.setPower(backLeftPower);
        rrMotor.setPower(backRightPower);

    }

    public void intake() {
        intakeDirection = gamepad2.left_trigger - gamepad2.right_trigger;

        if (intakeDirection > 0.0){
            intakeMotor.setPower(-1.0);
//            scoreMotor.setPower(0.2);
        }
        else if (intakeDirection < -0.1) {
            intakeMotor.setPower(1.0);
//            scoreMotor.setPower(-0.3);
        }
        else {
            intakeMotor.setPower(0.0);
            scoreMotor.setPower(0.0);
        }
    }

    public void scoreArt(){
        if (gamepad2.a) {
            if (currentHoodPosition > 0.2){
                scoreMotor.setPower(0.8);
            }
            else if (currentHoodPosition <= 0.2){
                scoreMotor.setPower(0.7);
            }

            telemetry.addData("Score Power", scoreMotor.getPower());
            telemetry.update();

        } else if (gamepad2.x) {
            scoreMotor.setPower(-0.3);
            intakeMotor.setPower(-1.0);
            telemetry.addData("Score Power", scoreMotor.getPower());
            telemetry.update();
        }
    }

    public void hood(){
        currentHoodPosition = Math.min(currentHoodPosition, 0.5);
        currentHoodPosition = Math.max(0.0, currentHoodPosition);
        adjustHood.setPosition(currentHoodPosition);
        if (gamepad2.dpad_down){
            currentHoodPosition += 0.1;
        }
        else if(gamepad2.dpad_up){
            currentHoodPosition -= 0.1;
        }
    }
}