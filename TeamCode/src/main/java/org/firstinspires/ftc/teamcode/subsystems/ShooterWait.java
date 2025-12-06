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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterWait extends SubsystemBase {
    private Servo AH;
    private DcMotorEx S;
    public static double close = 1000;
    public static double far = 1375;
    public static double HUp = 0.55;
    public static double HDown = 0.15;
    public static double HZero = 0.0;
    public static double intakePower = -150;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kP = 0.0;
    private double t = 0;
    private final Intake intakeSubsystem;
    private TelemetryManager telemetryM;
    private Timer actionTimer;
    private boolean actionIsRunning = false;

    public ShooterWait(HardwareMap hardwareMap, Intake intakeSubsystem) {
        S = hardwareMap.get(DcMotorEx.class, "SM");
        AH = hardwareMap.get(Servo.class, "AH");
        AH.setDirection(Servo.Direction.FORWARD);
        this.intakeSubsystem = intakeSubsystem;

        S.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public void stopMotor(){
        setTarget(0);
    }

    @Override
    public void periodic() {
        double currentVelocity = getVelocity();
        double targetVelocity = getTarget();

        double error = targetVelocity - currentVelocity;

        double power = (kV * targetVelocity) + (kP * error) + kS;

        S.setPower(power);

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

    public Command spinCloseCommand(){
        return new RunCommand(this::spinClose, this);
    }
    public Command intakein(){
        return new RunCommand(this::intake, this);
    }
    public Command spinFarCommand(){
        return new RunCommand(this::spinFar, this);
    }
    public Command stopCommand(){
        return new RunCommand(this::stopMotor, this);
    }

    public Command feedUpCommand(){
        return new RunCommand(this::feedUp);
    }

    public Command feedDownCommand(){
        return new RunCommand(this::feedDown);
    }
    public Command feedZeroCommand(){
        return new RunCommand(this::feedZero);
    }
    public boolean isAtVelocity(double targetVelocity) {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }
    public Command scoreFarCommand(){
        return new SequentialCommandGroup(
                spinFarCommand(),
                new WaitUntilCommand(() -> isAtVelocity(close)),
                spinFarCommand(),
                intakeSubsystem.inCommand(),
                new WaitCommand(3000),
                stopCommand(),
                intakeSubsystem.idleCommand()
        );
    }

    public Command scoreCloseCommand(){
        return new SequentialCommandGroup(
                spinCloseCommand()
                        .alongWith(
                                new WaitUntilCommand(() -> isAtVelocity(close))
                                        .andThen(intakeSubsystem.inCommand()),
                                new WaitCommand(5000)
                        )
                        .andThen(
                                stop(),
                                intakeSubsystem.idleCommand()
                        )
        );
    }

    public void getTelemetryData(Telemetry telemetry) {
        telemetry.addData("Shooter Velocity (actual)", S.getVelocity());
        telemetry.addData("Target Close Velocity", close);
        telemetry.addData("Target Far Velocity", far);
    }

    public Command stop(){
        return new SequentialCommandGroup(
                stopCommand()
        );
    }

    public boolean isAutoActionRunning() {
        return actionIsRunning;
    }
}
