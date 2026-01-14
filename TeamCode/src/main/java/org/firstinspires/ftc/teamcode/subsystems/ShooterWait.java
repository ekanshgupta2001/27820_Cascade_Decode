package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterWait extends SubsystemBase {
    private final Servo AH;     // Adjust hood / feeder servo
    private final Servo KS;     // Kick servo
    private final DcMotorEx S;  // Shooter motor

    // These should be POSITIVE values; we flip motor direction once, not targets.
    public static double close = 350;
    public static double far = 500;
    public static double medium = 425;
    public static double intakePower = 150;

    public static double HUp = 0.55;
    public static double HDown = 0.20;
    public static double HZero = 0.0;

    //These are our PIDF controls
    public static double kP = 0.00, kF = 11.7, kD = 0.000, kI = 0.000;

    public static double kup = 0.280;
    public static double kdown = 0.0;

    // This is the shooter target velocity we’re aiming for.
    private double targetVel = 0;

    // This decides if we’re actively trying to control shooter speed.
    public boolean activate = false;
    public boolean atVel = false;

    private final Intake intakeSubsystem;

    public ShooterWait(HardwareMap hardwareMap, Intake intakeSubsystem) {
        // This maps hardware names from RC config.
        S = hardwareMap.get(DcMotorEx.class, "SM");
        AH = hardwareMap.get(Servo.class, "AH");
        KS = hardwareMap.get(Servo.class, "KS");

        // This is just how your servo was set up before; keep if it matches your robot.
        AH.setDirection(Servo.Direction.REVERSE);

        KS.setDirection(Servo.Direction.FORWARD);

        this.intakeSubsystem = intakeSubsystem;

        // This sets motor behavior so velocity control is consistent.
        S.setDirection(DcMotorSimple.Direction.REVERSE);
        S.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, kF);
        S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

    }

    public void spinClose() {
        // This spins shooter to the close preset velocity.
        setTarget(close);
        feedDown();
    }

    public void spinMedium() {
        // This spins shooter to the far preset velocity.
        setTarget(medium);
        feedDown();
    }

    public void spinFar() {
        // This spins shooter to the far preset velocity.
        setTarget(far);
        feedZero();
    }

    public void intake() {
        // This spins shooter at a slower “intake/feed” speed.
        setTarget(intakePower);
    }

    public void setTarget(double velocity) {
        // This updates target velocity and enables shooter control immediately.
        targetVel = velocity;
        activate = (Math.abs(velocity) > 1e-6);
    }

    public double getTarget() {
        // This returns what we’re currently trying to hit.
        return targetVel;
    }

    public double getVelocity() {
        // This returns the actual motor velocity.
        return S.getVelocity();
    }

    public void stopMotor() {
        // This stops shooter cleanly and disables velocity control.
        setTarget(0);
        activate = false;
        S.setPower(0);
    }

    public void kickUp() {
        // This pushes the artifact into the shooter.
        KS.setPosition(kup);
    }

    public void kickDown() {
        // This retracts the kicker so the next artifact can stage.
        KS.setPosition(kdown);
    }

    public void feedUp() {
        // This moves the hood/feeder up.
        AH.setPosition(HUp);
    }

    public void feedDown() {
        // This moves the hood/feeder down.
        AH.setPosition(HDown);
    }

    public void feedZero() {
        // This sets hood/feeder to a “neutral” default.
        AH.setPosition(HZero);
    }

    public boolean isAtVelocity(double targetVelocity) {
        // This checks if we’re close enough to speed (tune the tolerance later).
        return Math.abs(targetVelocity - getVelocity()) < 50;
    }

    public void forDistance(double distanceX, double distanceY) {
        if (distanceX <= 28 && distanceY <= 31){
            spinClose();
        }
        else if (distanceX > 28 && distanceX <= 78 && distanceY > 31 && distanceY <= 67){
            spinMedium();
        }
        else if (distanceX > 78 && distanceY > 67){
            spinFar();
        }
        else {
            feedZero();
            stopMotor();
        }
    }

    @Override
    public void periodic() {
        if (activate) {
            S.setVelocity(targetVel);
        } else {
            S.setPower(0);
        }
    }


    public void getTelemetryData(Telemetry telemetry) {
        // This prints shooter info to help debug in matches.
        telemetry.addData("Shooter Velocity (actual)", S.getVelocity());
        telemetry.addData("Shooter Target", targetVel);
        telemetry.addData("Target Close", close);
        telemetry.addData("Target Far", far);
    }
}