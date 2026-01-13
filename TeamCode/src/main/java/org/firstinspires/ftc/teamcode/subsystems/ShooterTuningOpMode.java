package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;
import org.firstinspires.ftc.teamcode.subsystems.Intake; // Ensure this exists

@Config
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    // Allows you to toggle the shooter from the dashboard
    public static double testTargetVelocity = 415;
    public static boolean runShooter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dashboard telemetry to see graphs
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize subsystems (passing null for Intake if not needed for tuning)
        ShooterWait shooter = new ShooterWait(hardwareMap, null);

        telemetry.addLine("Ready. High-five the drive team.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Update Target based on Dashboard variable
            if (runShooter) {
                shooter.setTarget(testTargetVelocity);
            } else {
                shooter.stopMotor();
            }

            // 2. Run the periodic logic (Manual call since we aren't using a Command Scheduler loop here)
            shooter.periodic();

            // 3. Telemetry for Graphing
            // In FTC Dashboard, select these values to see the graph overlap
            telemetry.addData("01 - Target Velocity", shooter.getTarget());
            telemetry.addData("02 - Actual Velocity", shooter.getVelocity());
            telemetry.addData("03 - Error", shooter.getTarget() - shooter.getVelocity());
            telemetry.addData("04 - Motor Power", (ShooterWait.kF * shooter.getTarget()) + (ShooterWait.kP * (shooter.getTarget() - shooter.getVelocity())) + ShooterWait.kD);

            telemetry.update();
        }
    }
}
