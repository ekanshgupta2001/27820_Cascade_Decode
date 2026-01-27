package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;

@Config
@TeleOp(name = "PIDF Tuner", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    // Dashboard Controls - Change these in FTC Dashboard while running
    public static boolean RUN_SHOOTER = false;
    public static double TEST_TARGET = 400;

    // Quick preset buttons for common velocities
    public static boolean TEST_CLOSE = false;
    public static boolean TEST_MEDIUM = false;
    public static boolean TEST_FAR = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Telemetry to work with both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Subsystems
        Intake intake = new Intake(hardwareMap);
        ShooterWait shooter = new ShooterWait(hardwareMap, intake);

        telemetry.addLine("=================================");
        telemetry.addLine("  SHOOTER PIDF TUNING MODE");
        telemetry.addLine("=================================");
        telemetry.addLine("1. Open FTC Dashboard in browser");
        telemetry.addLine("2. Toggle RUN_SHOOTER to start");
        telemetry.addLine("3. Adjust TEST_TARGET velocity");
        telemetry.addLine("4. Tune kP, kI, kD, kF live");
        telemetry.addLine("=================================");
        telemetry.addLine("Ready! Press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle preset buttons
            if (TEST_CLOSE) {
                TEST_TARGET = ShooterWait.close;
                TEST_CLOSE = false;
            }
            if (TEST_MEDIUM) {
                TEST_TARGET = ShooterWait.medium;
                TEST_MEDIUM = false;
            }
            if (TEST_FAR) {
                TEST_TARGET = ShooterWait.far;
                TEST_FAR = false;
            }

            // Control shooter based on dashboard toggle
            if (RUN_SHOOTER) {
                shooter.setTarget(TEST_TARGET);
                // The setTarget method sets activate=true automatically
            } else {
                shooter.stopMotor();
            }

            // CRITICAL: Call periodic() to apply velocity control
            shooter.periodic();

            // Telemetry for dashboard graphs
            telemetry.addData("═══════════════════════════════", "");
            telemetry.addData("SHOOTER STATUS", RUN_SHOOTER ? "RUNNING ✓" : "STOPPED ✗");
            telemetry.addData("═══════════════════════════════", "");

            telemetry.addData("Target Velocity", "%.1f", shooter.getTarget());
            telemetry.addData("Actual Velocity", "%.1f", shooter.getVelocity());
            telemetry.addData("Error", "%.1f", shooter.getTarget() - shooter.getVelocity());
            telemetry.addData("At Speed?", shooter.isAtVelocity() ? "YES ✓" : "NO ✗");

            telemetry.addData("═══════════════════════════════", "");
            telemetry.addData("PIDF Coefficients", "");
            telemetry.addData("  kP", "%.2f", ShooterWait.kP);
            telemetry.addData("  kI", "%.2f", ShooterWait.kI);
            telemetry.addData("  kD", "%.2f", ShooterWait.kD);
            telemetry.addData("  kF", "%.2f", ShooterWait.kF);

            telemetry.addData("═══════════════════════════════", "");
            telemetry.addData("Controls", "");
            telemetry.addData("  RUN_SHOOTER", RUN_SHOOTER ? "ON" : "OFF");
            telemetry.addData("  TEST_TARGET", "%.0f", TEST_TARGET);

            // Additional detailed telemetry
            shooter.getTelemetryData(telemetry);

            telemetry.update();
        }

        // Stop shooter when opmode ends
        shooter.stopMotor();
    }
}