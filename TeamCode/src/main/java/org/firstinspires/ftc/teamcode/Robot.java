package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;

import java.util.function.Supplier;

public class Robot {
    public final Intake i;
    public final ShooterWait s;
    public final Webcam w;

    public Follower follower;
    public Alliance a;

    private Supplier<PathChain> parkBot;
    private final LynxModule hub;

    // This timer is just used for periodic things like caching.
    private final Timer loop = new Timer();

    public static Pose endPose;

    // These are the "base" blue poses. For RED we mirror them.
    private static final Pose BLUE_PARK_POSE = new Pose(43.53265717273622, 4.069461071585107, 101.881785924254);
    private static final Pose BLUE_SHOOT_TARGET = new Pose(6, 138, 0);

    public static Pose parkPose = BLUE_PARK_POSE;
    public static Pose shootTarget = BLUE_SHOOT_TARGET;

    public Robot(HardwareMap h, Telemetry telemetry, Alliance a) {
        // This sets the alliance right away so we don’t get null alliance bugs later.
        this.a = a;

        // These are the main subsystems: intake, shooter, and webcam.
        i = new Intake(h);
        w = new Webcam(h, telemetry, "Webcam 1");
        s = new ShooterWait(h, i);

        // This creates Pedro follower so we can drive / follow paths.
        follower = Constants.createFollower(h);

        // Manual bulk caching can be faster, but only if we clear it often.
        hub = h.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        loop.resetTimer();
        setShootTarget();
    }

    public void periodic() {
        // This clears bulk cache every loop so sensor reads don’t get stale.
        hub.clearBulkCache();

        // This updates target poses + follower + subsystems every loop.
        setShootTarget();
        follower.update();
        w.periodic();
        s.periodic();
    }

    public void stop() {
        // This saves final pose so we can debug / reuse it later.
        endPose = follower.getPose();
    }

    public void setShootTarget() {
        // This sets the target based on alliance using a clean “base pose then mirror” approach.
        if (a == Alliance.BLUE) shootTarget = BLUE_SHOOT_TARGET;
        else if (a == Alliance.RED) shootTarget = BLUE_SHOOT_TARGET.mirror();
    }

    public void setAlliance(Alliance newAlliance) {
        // This switches alliance and instantly updates any mirrored targets.
        this.a = newAlliance;
        setShootTarget();
    }

    public void park() {
        // This picks the correct park pose based on alliance.
        if (a == Alliance.BLUE) parkPose = BLUE_PARK_POSE;
        else if (a == Alliance.RED) parkPose = BLUE_PARK_POSE.mirror();

        // This builds a path starting from our CURRENT pose (so pose must not be null).
        parkBot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, parkPose)))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                0.8
                        )
                )
                .build();
    }

    public void startParkingPath() {
        // This is a safety check: if pose is null, Pedro will crash building the BezierLine.
        if (follower.getPose() == null) return;

        park();
        follower.followPath(parkBot.get());
    }

    public Pose getShootTarget() {
        // This returns whatever target pose is currently active.
        return shootTarget;
    }
}