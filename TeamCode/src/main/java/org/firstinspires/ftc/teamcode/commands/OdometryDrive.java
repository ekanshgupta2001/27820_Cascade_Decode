package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NonVisionRobot;

@Config
public class OdometryDrive {
    private DcMotor FL, FR, BL, BR;
    private GoBildaPinpointDriver pinpoint;

    // Tune these with FTC Dashboard
    public static double DRIVE_P = 0.05;
    public static double STRAFE_P = 0.05;
    public static double TURN_P = 0.02;

    public static double DRIVE_POWER = 0.5;  // Max power for driving
    public static double POSITION_TOLERANCE = 1.0;  // inches
    public static double ANGLE_TOLERANCE = 2.0;     // degrees

    public OdometryDrive(HardwareMap hardwareMap) {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(3.5, 3.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        // Set motor directions - ADJUST IF NEEDED
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setStartPosition(double startX, double startY, double startHeading) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, startX, startY,
                AngleUnit.DEGREES, startHeading
        ));
    }

    public void driveToPosition(double targetX, double targetY, double targetHeading, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < timeoutSeconds) {
            pinpoint.update();

            double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);
            double currentHeading = Math.toDegrees(pinpoint.getPosition().getHeading(AngleUnit.RADIANS));

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorHeading = targetHeading - currentHeading;

            // Normalize heading error to -180 to 180
            while (errorHeading > 180) errorHeading -= 360;
            while (errorHeading < -180) errorHeading += 360;

            // Check if we're at target
            if (Math.abs(errorX) < POSITION_TOLERANCE &&
                    Math.abs(errorY) < POSITION_TOLERANCE &&
                    Math.abs(errorHeading) < ANGLE_TOLERANCE) {
                break;
            }

            // Calculate powers with P control
            double xPower = errorX * STRAFE_P;
            double yPower = errorY * DRIVE_P;
            double turnPower = errorHeading * TURN_P;

            // Clip powers
            xPower = Math.max(-DRIVE_POWER, Math.min(DRIVE_POWER, xPower));
            yPower = Math.max(-DRIVE_POWER, Math.min(DRIVE_POWER, yPower));
            turnPower = Math.max(-DRIVE_POWER, Math.min(DRIVE_POWER, turnPower));

            // Mecanum drive kinematics
            double FLPower = yPower + xPower + turnPower;
            double FRPower = yPower - xPower - turnPower;
            double BLPower = yPower - xPower + turnPower;
            double BRPower = yPower + xPower - turnPower;

            // Normalize if any power exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(FLPower), Math.abs(FRPower)),
                    Math.max(Math.abs(BLPower), Math.abs(BRPower)));
            if (maxPower > 1.0) {
                FLPower /= maxPower;
                FRPower /= maxPower;
                BLPower /= maxPower;
                BRPower /= maxPower;
            }

            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }

        // Stop motors
        stopMotors();
    }

    public void stopMotors() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}