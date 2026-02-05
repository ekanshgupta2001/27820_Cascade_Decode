package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

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
    public static double close = 375;
    public static double far = 585;
    public static double medium = 470;
    public static double intakePower = 0.2;

    public static double HUp = 0.5;
    public static double HDown = 0.15;
    public static double HClose = 0.03;
    public static double HZero = 0.0;
    private final Timer spinupTimer = new Timer();
    private boolean justStartedSpinup = false;
    public static double kI_SPINUP = 0.7;   // High kI during initial spinup
    public static double kI_STEADY = 0.08;  //Regular kI values for the match
    public static double kP = 24;      // Proportional - how aggressively to correct error   24
    public static double kI = 0.08;      // Integral - eliminates steady-state error    0.08
    public static double kD = 14;      // Derivative - dampens oscillations     kD
    public static double kF = 25.8;      // Feedforward - calculated in constructor, but adjustable after   25.8

    public static boolean USE_MANUAL_KF = false;
    public static double velocityTolerance = 40;

    public static double kup = 0.280;
    public static double kdown = 0.0;
    private double targetVel = 0;
    public boolean activate = false;
    private final Intake intakeSubsystem;
    private PIDFCoefficients lastCoeffs = new PIDFCoefficients(kP, kI, kD, kF);

    public ShooterWait(HardwareMap hardwareMap, Intake intakeSubsystem) {
        // This maps hardware names from RC config.
        S = hardwareMap.get(DcMotorEx.class, "SM");
        AH = hardwareMap.get(Servo.class, "AH");
        KS = hardwareMap.get(Servo.class, "KS");

        AH.setDirection(Servo.Direction.REVERSE);
        KS.setDirection(Servo.Direction.REVERSE);

        this.intakeSubsystem = intakeSubsystem;

        S.setDirection(DcMotorSimple.Direction.REVERSE);
        S.setZeroPowerBehavior(FLOAT);

        // Double-reset encoder to clear any garbage data and integral buildup
        try {
            S.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            S.setPower(0);
            Thread.sleep(100);
            S.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(100);
            S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Zero PIDF first to clear any integral buildup
            S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(0, 0, 0, 0));
            Thread.sleep(50);

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Auto-calculate kF if not using manual value
//        if (!USE_MANUAL_KF) {
//            calculateFeedforward();
//        }

        updatePIDF();
    }

    private void updatePIDF() {
        lastCoeffs = new PIDFCoefficients(kP, kI, kD, kF);
        // This OVERRIDES the motor controller's default PIDF with our custom values
        S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastCoeffs);
    }
    private void setTargetWithBoost(double velocity) {
        targetVel = velocity;
        activate = true;
        justStartedSpinup = true;
        spinupTimer.resetTimer();
    }

    public void spinClose() {
        setTargetWithBoost(close);
        feedClose();
    }

    public void spinMedium() {
        setTargetWithBoost(medium);
        feedDown();
    }

    public void spinFar() {
        setTargetWithBoost(far);
        feedUp();
    }

    public void intake() {
        intakeSubsystem.set(intakePower);
    }

    public void setTarget(double velocity) {
        targetVel = velocity;
        activate = (Math.abs(velocity) > 1e-6);
    }

    public double getTarget() {
        return targetVel;
    }

    public double getVelocity() {
        return S.getVelocity();
    }

    public void stopMotor() {
        setTarget(0);
        activate = false;
        S.setPower(0);
    }

    public void kickUp() {
        KS.setPosition(kup);
    }

    public void kickDown() {
        KS.setPosition(kdown);
    }

    public void feedUp() {
        AH.setPosition(HUp);
    }

    public void feedDown() {
        AH.setPosition(HDown);
    }
    public void feedClose() {
        AH.setPosition(HClose);
    }

    public void feedZero() {
        AH.setPosition(HZero);
    }

    public boolean isAtVelocity(double targetVelocity) {
        // Use configurable tolerance
        return Math.abs(targetVelocity - getVelocity()) < velocityTolerance;
    }

    public boolean isAtVelocity() {
        // Convenience method using current target
        return isAtVelocity(targetVel);
    }

    // CLEANED UP: Removed unused distanceX parameter
    public void forDistance(double distanceY) {
        if (distanceY <= 40) spinClose();
        else if (distanceY <= 80) spinMedium();
        else spinFar();
    }

    @Override
    public void periodic() {
        if (justStartedSpinup && spinupTimer.getElapsedTime() < 750) {
            kI = kI_SPINUP;  // High kI for first 0.75 seconds
        } else {
            kI = kI_STEADY;  // Normal kI after that
            justStartedSpinup = false;
        }

        if (kP != lastCoeffs.p || kI != lastCoeffs.i ||
                kD != lastCoeffs.d || kF != lastCoeffs.f) {
            updatePIDF();
        }

        if (activate) {
            S.setVelocity(targetVel);
        } else {
            S.setPower(0);
        }
    }

    public void getTelemetryData(Telemetry telemetry) {
        telemetry.addData("Shooter Velocity (actual)", "%.1f", S.getVelocity());
        telemetry.addData("Shooter Target", "%.1f", targetVel);
        telemetry.addData("Error", "%.1f", targetVel - S.getVelocity());
        telemetry.addData("At Speed?", isAtVelocity() ? "YES" : "NO");
        telemetry.addData("Target Close/Med/Far", "%.0f / %.0f / %.0f", close, medium, far);
        telemetry.addData("Current PIDF", "P:%.1f I:%.1f D:%.1f F:%.1f",
                lastCoeffs.p, lastCoeffs.i, lastCoeffs.d, lastCoeffs.f);
        telemetry.addData("Motor Power", "%.2f", S.getPower());
    }
}