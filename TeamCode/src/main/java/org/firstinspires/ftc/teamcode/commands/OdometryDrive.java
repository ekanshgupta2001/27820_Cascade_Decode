//package org.firstinspires.ftc.teamcode.commands;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.seattlesolvers.solverslib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.NonVisionRobot;
//
//@Config
//public class OdometryDrive extends CommandBase {
//
//    private final NonVisionRobot r;
//    private final HardwareMap hardwareMap;
//    private final Timer timeoutTimer = new Timer();
//
//    private final Pose targetPose;  // Where we want to go
//    private final double maxPower;
//    private final double timeoutSeconds;
//    private final boolean maintainHeading;  // Keep current heading or turn to target?
//
//    // Motor references
//    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
//
//    // TUNE THESE values
//    public static double POSITION_TOLERANCE = 2.0;    // inches - how close is "close enough"
//    public static double HEADING_TOLERANCE = 5.0;     // degrees
//    public static double MIN_POWER = 0.1;             // Minimum power to move
//    public static double HEADING_kP = 0.02;           // Turn correction strength
//    public static double SLOW_DOWN_DISTANCE = 12.0;   // Start slowing down 12" before target
//
//    /**
//     * Drive to a target position using odometry
//     * @param r Robot
//     * @param hardwareMap Hardware map for motor access
//     * @param targetX Target X coordinate (inches)
//     * @param targetY Target Y coordinate (inches)
//     * @param targetHeading Target heading (degrees), or -999 to maintain current heading
//     * @param maxPower Maximum drive power
//     * @param timeoutSeconds Safety timeout
//     */
//    public OdometryDrive(NonVisionRobot r, HardwareMap hardwareMap, double targetX, double targetY,
//                         double targetHeading, double maxPower, double timeoutSeconds) {
//        this.r = r;
//        this.hardwareMap = hardwareMap;
//        this.targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
//        this.maxPower = Math.abs(maxPower);
//        this.timeoutSeconds = timeoutSeconds;
//        this.maintainHeading = (targetHeading == -999);
//    }
//
//    // Convenience constructors
//
//    /** Drive to position, maintain current heading */
//    public static OdometryDrive toPosition(NonVisionRobot r, HardwareMap hw,
//                                           double x, double y, double power) {
//        return new OdometryDrive(r, hw, x, y, -999, power, 10.0);
//    }
//
//    /** Drive to position with specific heading */
//    public static OdometryDrive toPose(NonVisionRobot r, HardwareMap hw,
//                                       double x, double y, double headingDegrees, double power) {
//        return new OdometryDrive(r, hw, x, y, headingDegrees, power, 10.0);
//    }
//
//    /** Drive forward relative to current position */
//    public static OdometryDrive forward(NonVisionRobot r, HardwareMap hw,
//                                        double inches, double power) {
//        Pose current = r.follower.getPose();
//        double targetX = current.getX() + inches * Math.cos(current.getHeading());
//        double targetY = current.getY() + inches * Math.sin(current.getHeading());
//        return new OdometryDrive(r, hw, targetX, targetY, -999, power, 10.0);
//    }
//
//    /** Strafe right relative to current position */
//    public static OdometryDrive strafeRight(NonVisionRobot r, HardwareMap hw,
//                                            double inches, double power) {
//        Pose current = r.follower.getPose();
//        double targetX = current.getX() + inches * Math.cos(current.getHeading() - Math.PI/2);
//        double targetY = current.getY() + inches * Math.sin(current.getHeading() - Math.PI/2);
//        return new OdometryDrive(r, hw, targetX, targetY, -999, power, 10.0);
//    }
//
//    @Override
//    public void initialize() {
//        // Get motor references from hardwareMap
//        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
//        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
//        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
//        backRight = hardwareMap.get(DcMotorEx.class, "BR");
//
//        timeoutTimer.resetTimer();
//    }
//
//    @Override
//    public void execute() {
//        // Update odometry
//        r.follower.update();
//
//        // Get current pose
//        Pose currentPose = r.follower.getPose();
//
//        // Calculate error (how far we are from target)
//        double deltaX = targetPose.getX() - currentPose.getX();
//        double deltaY = targetPose.getY() - currentPose.getY();
//        double distance = Math.hypot(deltaX, deltaY);
//
//        // Calculate angle to target (in robot frame)
//        double angleToTarget = Math.atan2(deltaY, deltaX);
//        double robotHeading = currentPose.getHeading();
//        double relativeAngle = angleToTarget - robotHeading;
//
//        // Normalize angle to -π to π
//        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
//        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
//
//        // Calculate drive components (in robot frame)
//        double forward = Math.cos(relativeAngle);
//        double strafe = Math.sin(relativeAngle);
//
//        // Calculate heading correction
//        double turn = 0;
//        if (!maintainHeading) {
//            double headingError = targetPose.getHeading() - robotHeading;
//            while (headingError > Math.PI) headingError -= 2 * Math.PI;
//            while (headingError < -Math.PI) headingError += 2 * Math.PI;
//            turn = headingError * HEADING_kP;
//        }
//
//        // Calculate power with ramp-down near target
//        double power;
//        if (distance < POSITION_TOLERANCE) {
//            forward = 0;
//            strafe = 0;
//        } else {
//            forward *= power;
//            strafe *= power;
//        }
//
//        // Scale drive components by power and distance
//        forward *= power;
//        strafe *= power;
//
//        // If very close, reduce drive power but keep turn correction
//        if (distance < POSITION_TOLERANCE) {
//            forward *= 0.1;
//            strafe *= 0.1;
//        }
//
//        // Calculate motor powers (mecanum drive equations)
//        double frontLeftPower = forward + strafe + turn;
//        double frontRightPower = forward - strafe - turn;
//        double backLeftPower = forward - strafe + turn;
//        double backRightPower = forward + strafe - turn;
//
//        // Normalize powers if any exceed 1.0
//        double maxMotorPower = Math.max(
//                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
//        );
//
//        if (maxMotorPower > 1.0) {
//            frontLeftPower /= maxMotorPower;
//            frontRightPower /= maxMotorPower;
//            backLeftPower /= maxMotorPower;
//            backRightPower /= maxMotorPower;
//        }
//
//        // Apply powers
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
//    }
//
//    @Override
//    public boolean isFinished() {
//        // Update odometry one last time
//        r.follower.update();
//        Pose currentPose = r.follower.getPose();
//
//        // Calculate position error
//        double deltaX = targetPose.getX() - currentPose.getX();
//        double deltaY = targetPose.getY() - currentPose.getY();
//        double distance = Math.hypot(deltaX, deltaY);
//
//        // Calculate heading error (if we care about it)
//        boolean headingGood = true;
//        if (!maintainHeading) {
//            double headingError = Math.toDegrees(targetPose.getHeading() - currentPose.getHeading());
//            while (headingError > 180) headingError -= 360;
//            while (headingError < -180) headingError += 360;
//            headingGood = Math.abs(headingError) < HEADING_TOLERANCE;
//        }
//
//        boolean positionGood = distance < POSITION_TOLERANCE;
//        boolean timedOut = timeoutTimer.getElapsedTime() >= timeoutSeconds;
//
//        return (positionGood && headingGood) || timedOut;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        // Stop all motors
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
//}