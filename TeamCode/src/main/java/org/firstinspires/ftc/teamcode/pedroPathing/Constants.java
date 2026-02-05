package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.6)
            .lateralZeroPowerAcceleration(-57.188979798647296)
            .forwardZeroPowerAcceleration(-41.278)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.01, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0001, 0.6, 0.0147 ))
            .centripetalScaling(0.00045);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(67.92431832861713)
            .yVelocity(55.77887972133366);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)  //FIXME: Check if in the correct direction
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);  //FIXME: Check if in the correct direction

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            0.9,
            0.95
    );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
