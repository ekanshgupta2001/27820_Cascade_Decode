package org.firstinspires.ftc.teamcode.subsystems;

// Import the necessary FTCLib components

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    private final DcMotorEx i;
    private final DcMotorEx f;
    public static double idle = 0;
    public static double in = -1;
    public static double out = 1;

    public static double inf = 1.0;
    public static double outf = -1.0;
    public static double idlef = 0.0;

    public Intake(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "IM");
        i.setDirection(DcMotorSimple.Direction.FORWARD);

        f = hardwareMap.get(DcMotorEx.class, "FM");
        f.setDirection(DcMotorSimple.Direction.FORWARD);

        set(0);
        setfunnel(0);
    }

    public void set(double power) {
        i.setPower(power);
    }

    public void setfunnel(double fpower){
        f.setPower(fpower);
    }

    public void spinIn() {
        set(in);
        setfunnel(inf);
    }

    public void spinOut() {
        set(out);
        setfunnel(outf);
    }

    public void spinIdle() {
        set(idle);
        setfunnel(idlef);
    }

    public Command idleCommand() {
        // Use the correct FTCLib class
        return new RunCommand(this::spinIdle, this);
    }

    public Command inCommand() {
        return new RunCommand(this::spinIn, this);
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
        telemetry.addData("Funnel Speed", f.getPower());
    }
}
