package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;

@Config
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    // Controls for Dashboard
    public static boolean RUN_SHOOTER = false;
    public static double TEST_TARGET = 400;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Telemetry to work with both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Subsystems (Adjust constructor if your Intake needs different params)
        Intake intake = new Intake(hardwareMap);
        ShooterWait shooter = new ShooterWait(hardwareMap, intake);

        telemetry.addLine("Ready to tune!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update the shooter's periodic logic (which handles PIDF updates)
            shooter.periodic();

            shooter.setTarget(TEST_TARGET);

            // Output data for graphing
            telemetry.addData("01 - Target Velocity", shooter.getTarget());
            telemetry.addData("02 - Actual Velocity", shooter.getVelocity());
            telemetry.addData("03 - Velocity Error", shooter.getTarget() - shooter.getVelocity());

            shooter.getTelemetryData(telemetry);
            telemetry.update();
        }
    }
}
