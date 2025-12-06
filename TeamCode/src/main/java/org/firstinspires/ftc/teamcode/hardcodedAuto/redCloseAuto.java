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

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class redCloseAuto extends LinearOpMode {
    private Follower follower;
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
        hood(0.6);
        intake(0.5, 400);
        move(1300, -0.5);
        sleep(1000);
        shoot(0.6, 0.7, 2000);
        sleep(3000);
        move(1400, 0.5);
        intake(-1.0, 2000);
        sleep(3000);
        move(1300, -0.5);
        sleep(1000);
        shoot(0.5, 0.7, 2000);
        sleep(3000);
        moveSideways(0.5, 850);
        hood(0.0);

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

    public void shoot(double angle, double speed, long time){
        scoreMotor.setPower(speed);
        adjustHood.setPosition(angle);
        sleep(5000);
        intakeMotor.setPower(-1.0);
        scoreMotor.setPower(speed);
        sleep(time);

        intakeMotor.setPower(0.0);
        adjustHood.setPosition(0.0);
        scoreMotor.setPower(0.0);
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

    public void hood(double angle){
        adjustHood.setPosition(angle);
    }




}
