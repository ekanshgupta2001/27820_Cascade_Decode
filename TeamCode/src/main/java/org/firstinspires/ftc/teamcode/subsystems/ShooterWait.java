package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterWait extends SubsystemBase {
    private Servo AH;
    private DcMotorEx S;
    private ServoEx KS;
    public static double close = -350;
    public static double far = -600;
    public static double HUp = 0.55;
    public static double HDown = 0.15;
    public static double HZero = 0.0;
    public static double kup = 50;
    public static double kdown = 0.0;
    public static double intakePower = -150;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kP = 0.0;
    public static boolean activate = false;
    private double t = 0;
    private final Intake intakeSubsystem;
    private TelemetryManager telemetryM;
    private Timer actionTimer;
    private boolean actionIsRunning = false;

    public ShooterWait(HardwareMap hardwareMap, Intake intakeSubsystem) {
        S = hardwareMap.get(DcMotorEx.class, "SM");
        AH = hardwareMap.get(Servo.class, "AH");
        AH.setDirection(Servo.Direction.FORWARD);
        KS = new ServoEx(hardwareMap, "KS", kup, kdown);

        this.intakeSubsystem = intakeSubsystem;

        S.setDirection(DcMotorSimple.Direction.REVERSE);
        S.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spinClose(){
        setTarget(close);
    }
    public void setTarget(double velocity) {
        t = velocity;
    }
    public double getTarget() {
        return t;
    }
    public double getVelocity() {
        return S.getVelocity();
    }
    public void spinFar(){
        setTarget(far);
    }
    public void intake(){
        setTarget(intakePower);
    }

    public void kickUp(){
        KS.set(kup);
    }
    public void kickDown(){
        KS.set(kdown);
    }

    public void stopMotor(){
        setTarget(0);
        activate = false;
    }

    @Override
    public void periodic() {
        double currentVelocity = getVelocity();
        double targetVelocity = getTarget();
        double power = ((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);

        if (activate) {
            S.setPower(power);
        }



        if (telemetryM != null) {
            telemetryM.addData("Shooter Target", targetVelocity);
            telemetryM.addData("Shooter Actual", currentVelocity);
            telemetryM.addData("Calculated Power", power);
        }
    }

    public void feedUp(){
        AH.setPosition(HUp);
    }

    public void feedDown(){
        AH.setPosition(HDown);
    }
    public void feedZero(){
        AH.setPosition(HZero);
    }
    public boolean isAtVelocity(double targetVelocity) {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }

    public void forDistance(double distance){
        //THIS NEEDS TO BE TUNED, A, B and C values need to be figured out
        setTarget((Math.pow(distance, 2))+(distance));
        activate = true;
    }

    public void getTelemetryData(Telemetry telemetry) {
        telemetry.addData("Shooter Velocity (actual)", S.getVelocity());
        telemetry.addData("Target Close Velocity", close);
        telemetry.addData("Target Far Velocity", far);
    }
}
