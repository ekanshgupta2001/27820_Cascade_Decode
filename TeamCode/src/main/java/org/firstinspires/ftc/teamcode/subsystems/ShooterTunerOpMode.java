package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTunerOpMode extends OpMode {

    // This is the velocity we’re trying to hit (ticks/sec for getVelocity()).
    public static double TARGET_VELOCITY = 600;

    // This switches between using setVelocity() (built-in PIDF) vs manual setPower math.
    public static boolean USE_BUILT_IN_VELOCITY = true;

    // These PIDF values are for the motor’s built-in velocity controller.
    public static double VEL_kP = 0.0;
    public static double VEL_kI = 0.0;
    public static double VEL_kD = 0.0;
    public static double VEL_kF = 0.0;

    // These are for manual feedforward + P control (only used if USE_BUILT_IN_VELOCITY = false).
    public static double MAN_kS = 0.0;
    public static double MAN_kV = 0.0;
    public static double MAN_kP = 0.0;

    // This clamps power so we don’t accidentally send something insane.
    public static double MAX_POWER = 1.0;

    private DcMotorEx motorS;
    private Telemetry tele;

    @Override
    public void init() {
        // This sets up telemetry for Driver Station + Dashboard at the same time.
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // This grabs the shooter motor once (don’t re-get it every loop).
        motorS = hardwareMap.get(DcMotorEx.class, "SM");

        // This makes sure velocity readings actually work.
        motorS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tele.addData("Status", "Initialized. Open FTC Dashboard and tune values.");
        tele.update();
    }

    @Override
    public void start() {
        // This is just a clean start so we don’t keep old power/velocity commands.
        motorS.setPower(0);
    }

    @Override
    public void loop() {
        // This is the current measured velocity (ticks/sec).
        double actualVel = motorS.getVelocity();
        double targetVel = TARGET_VELOCITY;

        if (USE_BUILT_IN_VELOCITY) {
            // This uses the built-in motor controller PIDF for velocity control.
            motorS.setVelocityPIDFCoefficients(VEL_kP, VEL_kI, VEL_kD, VEL_kF);
            motorS.setVelocity(targetVel);
        } else {
            // This is a simple manual controller: kS + kV*target + kP*error -> power.
            double error = targetVel - actualVel;
            double power = (MAN_kV * targetVel) + (MAN_kP * error) + MAN_kS;
            power = clip(power, -MAX_POWER, MAX_POWER);
            motorS.setPower(power);
        }

        // This prints the stuff you actually care about while tuning.
        tele.addData("Mode", USE_BUILT_IN_VELOCITY ? "Built-in setVelocity()" : "Manual setPower()");
        tele.addData("Target Vel", targetVel);
        tele.addData("Actual Vel", actualVel);
        tele.addData("Error", targetVel - actualVel);

        tele.addData("BuiltIn kP/kI/kD/kF", "%.4f  %.4f  %.4f  %.4f", VEL_kP, VEL_kI, VEL_kD, VEL_kF);
        tele.addData("Manual kS/kV/kP", "%.4f  %.6f  %.6f", MAN_kS, MAN_kV, MAN_kP);
        tele.addData("Max Power", MAX_POWER);

        tele.update();
    }

    @Override
    public void stop() {
        // This stops the motor at the end so it doesn’t keep spinning.
        motorS.setPower(0);
    }

    // This is a tiny helper so we keep values inside a safe range.
    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}