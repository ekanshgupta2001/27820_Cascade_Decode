package org.firstinspires.ftc.teamcode.decode_auto.plz_auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NonVisionRobot;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.OdometryDrive;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.decode_teleop.tele2Manual;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;

@Autonomous
public class pinRedClose extends LinearOpMode {
    private Follower follower;
    NonVisionRobot r;
    private OdometryDrive drive;
    private ShooterWait shooter;
    //    private enum AutoShootState {
//        IDLE,           // Not shooting
//        SPINNING_UP,    // Flywheel spinning to target velocity
//        READY,          // At velocity, waiting for trigger
//        SHOOTING,       // Kicker up, releasing sample
//        RESETTING,      // Kicker down, feeding next sample
//        COMPLETE        // All 3 shots done
//    }
//    private AutoShootState autoState = AutoShootState.IDLE;
    private final Timer autoTimer = new Timer();
    private final Timer intakeTimer = new Timer();
    public static double AUTO_SPINUP_MIN_TIME = 1.0;  // Minimum time to spin up
    public static double AUTO_KICK_UP_TIME = 0.25;    // How long kicker stays up
    public static double AUTO_RESET_TIME = 0.4;
    private double currentPower = 0;
    private double adjustSpeed = 0.5;
    private double intakeDirection = 0.0;
    private double lastTime;
    private int lastPosition = 0;
    private double velocity;
    private double timeCheck;
    private boolean slowModeActive = false;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        drive.setStartPosition(21.913, 123, 136);
        autoTimer.resetTimer();

        Auto();
        sleep(100000);
        telemetry.addData("Auto phase", "Done");
        telemetry.update();
    }
    public void Auto(){
        new Shoot(r);
        drive.driveToPosition(45, 84, 136, 3.0);
        if (autoTimer.getElapsedTime() > 4000){
            new IntakeIn(r.i);
            drive.driveToPosition(16.5, 84, 180, 3.0);
            r.s.forDistance(60);
            if (autoTimer.getElapsedTime() > 8000){
                drive.driveToPosition(45, 84, 136, 3.0);
                sleep(1200);
                new Shoot(r);
                if (autoTimer.getElapsedTime() > 12000){
                    drive.driveToPosition(30, 84, 136, 3.0);
                }
            }
        }
    }

}
