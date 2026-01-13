package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;

import java.util.List;

public class NonVisionRobot {
    public final Intake i;
    public final ShooterWait s;

    public final Follower follower;
    public Alliance a;

    private final LynxModule hub;          // Used for manual bulk caching (optional but fast).
    private final boolean hasHub;          // Safety flag in case hub list is empty for any reason.

    // Timer for periodic / caching rhythms if you ever need it.
    private final Timer loop = new Timer();

    // Useful for debugging final pose after OpMode ends.
    public static Pose endPose;

    // Base blue target pose. For red, we mirror it.
    private static final Pose BLUE_SHOOT_TARGET = new Pose(10, 138, 0);
    public static Pose shootTarget = BLUE_SHOOT_TARGET;

    public NonVisionRobot(HardwareMap h, Telemetry telemetry, Alliance a) {
        // Alliance set immediately to avoid null logic later.
        this.a = a;

        // Subsystems (NO WEBCAM).
        i = new Intake(h);
        s = new ShooterWait(h, i);

        // Pedro follower (localization/path following).
        follower = Constants.createFollower(h);

        // Manual bulk caching setup (safe).
        LynxModule tempHub = null;
        boolean tempHasHub = false;

        List<LynxModule> hubs = h.getAll(LynxModule.class);
        if (hubs != null && !hubs.isEmpty()) {
            tempHub = hubs.get(0);
            tempHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            tempHasHub = true;
        }

        hub = tempHub;
        hasHub = tempHasHub;

        loop.resetTimer();
        setShootTarget();
    }

    /**
     * Call once per loop in your OpMode.
     * Keeps localization updated and runs subsystem periodic methods.
     */
    public void periodic() {
        // Clear bulk cache each loop so reads stay fresh (only if hub exists).
        if (hasHub) hub.clearBulkCache();

        // Keep target updated in case alliance changes mid-run.
        setShootTarget();

        // Update follower + subsystems.
        follower.update();
        s.periodic();
    }

    /**
     * Call in OpMode stop/end to save pose for debugging.
     */
    public void stop() {
        endPose = follower.getPose();
    }

    /**
     * Updates the shoot target pose based on alliance (mirrors for red).
     */
    public void setShootTarget() {
        if (a == Alliance.BLUE) shootTarget = BLUE_SHOOT_TARGET;
        else if (a == Alliance.RED) shootTarget = BLUE_SHOOT_TARGET.mirror();
        else shootTarget = BLUE_SHOOT_TARGET; // Failsafe
    }

    /**
     * Allows switching alliance on the fly (telemetry menu / prestart selection).
     */
    public void setAlliance(Alliance newAlliance) {
        this.a = newAlliance;
        setShootTarget();
    }

    /**
     * Returns the currently active shooting target pose.
     */
    public Pose getShootTarget() {
        return shootTarget;
    }
}