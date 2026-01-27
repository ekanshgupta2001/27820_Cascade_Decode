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

    // These should be POSITIVE values; we flip motor direction once, not targets.
    public static double close = 300;
    public static double far = 475;
    public static double medium = 375;
    public static double intakePower = 0.2;

    public static double HUp = 0.48;
    public static double HDown = 0.20;
    public static double HZero = 0.0;

    // PIDF VALUES - These will be tuned via FTC Dashboard
    // kF gets auto-calculated on first run, then you can manually adjust all values
    public static double kP = 0.0;      // Proportional - how aggressively to correct error
    public static double kI = 0.0;      // Integral - eliminates steady-state error
    public static double kD = 0.0;      // Derivative - dampens oscillations
    public static double kF = 0.0;      // Feedforward - calculated in constructor, but adjustable after

    // Set this to true to skip auto-calculation and use manual kF value above
    public static boolean USE_MANUAL_KF = false;

    // Velocity tolerance for "at speed" check (ticks/sec)
    public static double velocityTolerance = 25;

    public static double kup = 0.280;
    public static double kdown = 0.0;

    // This is the shooter target velocity we're aiming for.
    private double targetVel = 0;

    // This decides if we're actively trying to control shooter speed.
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

        // CRITICAL FIX: Use RUN_USING_ENCODER to enable velocity control
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
            // If interrupted, just continue - not critical
            Thread.currentThread().interrupt();
        }

        // Auto-calculate kF if not using manual value
        if (!USE_MANUAL_KF) {
            calculateFeedforward();
        }

        // Apply our custom PIDF coefficients (overrides motor controller defaults)
        updatePIDF();
    }

    /**
     * Auto-calculates feedforward coefficient by measuring motor's max velocity.
     * This gets you 90% of the way there - the other PIDF terms fine-tune it.
     * After first run, you can see the calculated value in Dashboard and manually adjust if needed.
     */
    private void calculateFeedforward() {
        // Briefly run motor at full power to measure max velocity
        S.setPower(1.0);
        try {
            Thread.sleep(150); // Let it spin up
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        double maxVelocity = S.getVelocity();
        S.setPower(0.0);

        // kF formula: 32767 / max_velocity
        // 32767 is the internal max "power" value used by motor controller
        if (maxVelocity > 10) {  // Safety check to avoid division by zero
            kF = 32767.0 / maxVelocity;
        } else {
            // Fallback if measurement fails (motor didn't spin)
            kF = 12.0;  // Conservative starting estimate
        }
    }

    private void updatePIDF() {
        lastCoeffs = new PIDFCoefficients(kP, kI, kD, kF);
        // This OVERRIDES the motor controller's default PIDF with our custom values
        S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastCoeffs);
    }

    public void spinClose() {
        setTarget(close);
        feedDown();
    }

    public void spinMedium() {
        setTarget(medium);
        feedDown();
    }

    public void spinFar() {
        setTarget(far);
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

    // DEPRECATED: Keep old signature for compatibility, but redirect
    @Deprecated
    public void forDistance(double distanceX, double distanceY) {
        forDistance(distanceY);  // Just use Y distance
    }

    @Override
    public void periodic() {
        // Update PIDF if tuning via dashboard
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