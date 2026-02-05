package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class OdometryDrive {
    private DcMotorEx FL, FR, BL, BR;
    private GoBildaPinpointDriver pinpoint;

    // Tune these with FTC Dashboard
    public static double DRIVE_P = 0.05;
    public static double STRAFE_P = 0.06; // Strafe usually needs more power due to friction
    public static double TURN_P = 0.03;

    public static double MAX_POWER = 0.7;
    public static double POSITION_TOLERANCE = 1.0;  // inches
    public static double ANGLE_TOLERANCE = 2.0;     // degrees

    public OdometryDrive(HardwareMap hardwareMap) {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        // Directions: Adjust so that positive power = forward movement
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // 1. Check your offsets! These are the distance from center of rotation to the pods.
        pinpoint.setOffsets(3.5, 3.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
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

            Pose2D currentPos = pinpoint.getPosition();
            double currentX = currentPos.getX(DistanceUnit.INCH);
            double currentY = currentPos.getY(DistanceUnit.INCH);
            double currentHeadingRad = currentPos.getHeading(AngleUnit.RADIANS);
            double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

            // 1. Calculate Field-Centric Errors
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorHeading = targetHeading - currentHeadingDeg;

            // Normalize heading error to -180 to 180
            while (errorHeading > 180) errorHeading -= 360;
            while (errorHeading < -180) errorHeading += 360;

            // Exit condition
            if (Math.hypot(errorX, errorY) < POSITION_TOLERANCE && Math.abs(errorHeading) < ANGLE_TOLERANCE) {
                break;
            }

            // 2. THE ROTATION MATRIX
            // This converts field-centric errors into robot-centric commands
            // so the robot moves toward the target regardless of which way it is facing.
            double robotRelativeX = errorX * Math.cos(-currentHeadingRad) - errorY * Math.sin(-currentHeadingRad);
            double robotRelativeY = errorX * Math.sin(-currentHeadingRad) + errorY * Math.cos(-currentHeadingRad);

            // 3. Calculate powers with P control
            double xPower = robotRelativeX * STRAFE_P;
            double yPower = robotRelativeY * DRIVE_P;
            double turnPower = errorHeading * TURN_P;

            // 4. Mecanum Kinematics (Robot-Centric)
            // Note: xPower is strafe, yPower is forward/backward
            double flPower = yPower + xPower + turnPower;
            double frPower = yPower - xPower - turnPower;
            double blPower = yPower - xPower + turnPower;
            double brPower = yPower + xPower - turnPower;

            // 5. Normalize and Clip
            double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                    Math.max(Math.abs(blPower), Math.max(Math.abs(brPower), 1.0))));

            // Apply MAX_POWER limit from Dashboard
            double scale = MAX_POWER / max;

            FL.setPower(flPower * scale);
            FR.setPower(frPower * scale);
            BL.setPower(blPower * scale);
            BR.setPower(brPower * scale);
        }

        stopMotors();
    }

    public void stopMotors() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
