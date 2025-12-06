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

import java.util.ArrayList;
import java.util.List;

public class Webcam{
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private final Telemetry telemetry;
    private int targetTagID = -1;

    public Webcam(HardwareMap hwMap, Telemetry telemetry){
        this(hwMap, telemetry, "Webcam 1");
    }

    public Webcam(HardwareMap hwMap, Telemetry telemetry, String cameraName){
        this.telemetry = telemetry;
        initCamera(hwMap, cameraName);
    }
    public void initCamera(HardwareMap hwMap, String cameraName){
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true) //Shows what the ID actually is
                .setDrawTagOutline(true)
                .setDrawAxes(true) //Draws the Axes
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES) //Gets the units
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 0, 8, 13, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 15, 0, 0)
                )
                .build();

        aprilTagProcessor.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, cameraName));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
        visionPortal.resumeStreaming();
    }

    public void periodic(){
        if (aprilTagProcessor != null){
            detectedTags = aprilTagProcessor.getDetections();
        }
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificID(int id){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public void setTargetTagID(int id){
        targetTagID = id;
    }

    public int getTargetTagID(){
        return targetTagID;
    }

    public AprilTagDetection getTargetTag(){
        if (targetTagID < 0) return null;
        return getTagBySpecificID(targetTagID);
    }

    public double getDistancetoTagId(){
        AprilTagDetection tag = getTargetTag();
        if (tag == null || tag.ftcPose == null){
            return -1;
        }
        return tag.ftcPose.y;
    }

    public void displayTagTelemetry(AprilTagDetection detection){
        if (detection == null){
            telemetry.addLine("No Tag found");
            return;
        }

        if (detection.metadata != null){
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (CM)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (CM, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }

    }

    public void stop(){
        if (visionPortal != null){
            visionPortal.close();
            visionPortal = null;
        }
    }


}
