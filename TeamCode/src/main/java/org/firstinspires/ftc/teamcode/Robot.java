package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
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
    private final Timer loop = new Timer();
    public static Pose endPose;
    public static Pose parkPose = new Pose(43.53265717273622, 4.069461071585107, 101.881785924254);;
    public static Pose shootTarget = new Pose(6, 138, 0);

    public Robot(HardwareMap h, Telemetry telemetry, Alliance a) {
        i = new Intake(h);
        w = new Webcam(h, telemetry, "Webcam 1");
        s = new ShooterWait(h, i);
        follower = Constants.createFollower(h);

        hub = h.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        loop.resetTimer();
        setShootTarget();

    }

    public void periodic() {
        setShootTarget();

        if (loop.getElapsedTime() >= 5.0) {
            hub.clearBulkCache();
            loop.resetTimer();
        }

        follower.update();
        w.periodic();
        s.periodic();
    }

    public void stop() {
        endPose = follower.getPose();
    }

    public void setShootTarget() {
        if (a == Alliance.BLUE)
            shootTarget = new Pose(6, 138, 0);
        else if (a == Alliance.RED)
            shootTarget = shootTarget.mirror();
    }

    public void setAlliance(Alliance newAlliance) {
        this.a = newAlliance;
        setShootTarget();
    }

    public void park(){
        if (a == Alliance.BLUE){
            parkPose = new Pose(43.53265717273622, 4.069461071585107, 101.881785924254);
        }
        else if (a == Alliance.RED){
            parkPose.mirror();
        }

        parkBot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, parkPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();
    }

    public void startParkingPath() {
        park();

        follower.followPath(parkBot.get());
    }


    public Pose getShootTarget() {
        return shootTarget;
    }
}
