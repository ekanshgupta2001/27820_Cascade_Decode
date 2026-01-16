package org.firstinspires.ftc.teamcode.subsystems;

// Import the necessary FTCLib components

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    private final DcMotorEx i;
    public static double idle = 0;
    public static double in = -0.9;
    public static double shooterSpin = -0.4;
    public static double out = 0.9;

    public Intake(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "IM");
        i.setDirection(DcMotorSimple.Direction.FORWARD);
        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        set(0);
    }

    public void set(double power) {
        i.setPower(power);
    }

    public void spinIn() {
        set(in);
    }

    public void intakeShooter() {
        set(shooterSpin);
    }

    public void spinOut() {
        set(out);
    }

    public void spinIdle() {
        set(idle);
    }

    public Command idleCommand() {
        // Use the correct FTCLib class
        return new RunCommand(this::spinIdle, this);
    }

    public Command inCommand() {
        return new RunCommand(this::spinIn, this);
    }

    public Command shooterinCommand() {
        return new RunCommand(this::intakeShooter, this);
    }

    public Command outCommand() {
        return new RunCommand(this::spinOut, this);
    }

    public Command stopCommand() {
        // Use the correct FTCLib class
        return new RunCommand(this::spinIdle, this);
    }

    public void getTelemetryData(Telemetry telemetry) {
        telemetry.addData("Shooter Velocity (actual)", i.getPower());
    }
}
