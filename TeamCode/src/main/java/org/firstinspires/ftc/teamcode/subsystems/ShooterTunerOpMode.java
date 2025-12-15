package org.firstinspires.ftc.teamcode.subsystems; // Adjust this package name to match your project

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// The @Config annotation allows you to tune the static variables in Shooter.java
// directly from the FTC Dashboard web interface while the robot is running.
@Config
@TeleOp(name = "Shooter Tuner")
public class ShooterTunerOpMode extends CommandOpMode {

    // You can add static variables here if you want to tune the *target* velocity
    // from this OpMode specifically, otherwise it uses the 'far' constant defined in Shooter.java
    public static double TUNE_TARGET_VELOCITY = -800;

    private ShooterWait shooter;

    // We create a local instance of Telemetry to send data to the Dashboard and Driver Station
    private Telemetry telemetryDS;

    @Override
    public void initialize() {
        // Initialize standard FTCLib OpMode components
        telemetryDS = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        // We can't fully initialize the Shooter subsystem because we don't have
        // an 'Intake' subsystem handy in this specific tuner OpMode.
        // Instead, we will grab the motor reference manually to run the control loop.

        // Initialize hardware manually for simple tuning purposes
        DcMotorEx motorS = hardwareMap.get(DcMotorEx.class, "SM");
        motorS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // You'll need to initialize your Shooter class slightly differently if you want to use
        // the full class structure. For a simple tuner, running the loop manually is easier.

        // We will manage the control loop logic directly in the run() method of this OpMode
        // instead of relying on the SubsystemBase's periodic() method here.

        // Initialize our target velocity
        Shooter.far = TUNE_TARGET_VELOCITY;

        telemetryDS.addData("Status", "Initialized. Connect to FTC Dashboard to tune values.");
        telemetryDS.update();
    }

    @Override
    public void run() {
        // This runs repeatedly after INIT is pressed and then START

        // Manual implementation of the control loop for tuning feedback:

        DcMotorEx motorS = hardwareMap.get(DcMotorEx.class, "SM"); // Re-access motor here

        double currentVelocity = motorS.getVelocity();
        double targetVelocity = TUNE_TARGET_VELOCITY;

        double error = targetVelocity - currentVelocity;

        // Calculate the power using the static K values defined in your Shooter class
        double power = (ShooterWait.kV * targetVelocity) + (ShooterWait.kP * error) + ShooterWait.kS;

        // Apply the calculated power
        motorS.setPower(power);

        // Send data to the Driver Station and FTC Dashboard
        telemetryDS.addData("Target Velocity", targetVelocity);
        telemetryDS.addData("Actual Velocity", currentVelocity);
        telemetryDS.addData("Calculated Power", power);
        telemetryDS.addData("kP", ShooterWait.kP);
        telemetryDS.addData("kV", ShooterWait.kV);
        telemetryDS.addData("kS", ShooterWait.kS);
        telemetryDS.update();
    }
}
