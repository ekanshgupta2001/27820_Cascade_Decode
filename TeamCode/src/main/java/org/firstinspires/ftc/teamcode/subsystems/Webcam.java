package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class Webcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private final Telemetry telemetry;
    private int targetTagID = -1;

    public Webcam(HardwareMap hwMap, Telemetry telemetry) {
        this(hwMap, telemetry, "Webcam 1");
    }

    public Webcam(HardwareMap hwMap, Telemetry telemetry, String cameraName) {
        this.telemetry = telemetry;
        initCamera(hwMap, cameraName);
    }

    public void initCamera(HardwareMap hwMap, String cameraName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // Position: X, Y, Z from robot center (Inches)
                // Orientation: Yaw, Pitch, Roll (Sideways mount = 90 roll)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 0, 8, 13, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 15, 90, 0)
                )
                .build();

        aprilTagProcessor.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, cameraName));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void periodic() {
        if (aprilTagProcessor != null) {
            detectedTags = aprilTagProcessor.getDetections();
        }
    }

    public boolean isTagVisible(int id) {
        return getTagBySpecificID(id) != null;
    }

    /**
     * Calculates Robot Pose for Pedro Pathing.
     * Converts FTC Field Center (0,0) to Pedro Field Corner (0,0) by adding 72 inches.
     */
    public Pose getRobotPoseFieldSpace(int id) {
        AprilTagDetection detection = getTagBySpecificID(id);
        if (detection != null && detection.robotPose != null) {
            return new Pose(
                    detection.robotPose.getPosition().x + 72.0,
                    detection.robotPose.getPosition().y + 72.0,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
            );
        }
        return null;
    }

    /**
     * Returns physical straight-line distance to tag for shooter velocity.
     */
    public double getRangeToTag() {
        AprilTagDetection tag = getTargetTag();
        return (tag != null && tag.ftcPose != null) ? tag.ftcPose.range : -1;
    }

    public AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void setTargetTagID(int id) {
        targetTagID = id;
    }

    public int getTargetTagID() {
        return targetTagID;
    }

    public AprilTagDetection getTargetTag() {
        if (targetTagID < 0) return null;
        return getTagBySpecificID(targetTagID);
    }

    public void displayTagTelemetry(AprilTagDetection detection) {
        if (detection == null) {
            telemetry.addLine("No Tag found");
            return;
        }

        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (IN)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (IN, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
