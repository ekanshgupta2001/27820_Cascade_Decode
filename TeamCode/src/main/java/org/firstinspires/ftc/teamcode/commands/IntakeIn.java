package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class IntakeIn extends CommandBase {
    private final Intake i;
    private final Timer t = new Timer();

    public static double INTAKE_TIME = 1.5; // seconds

    public IntakeIn(Intake i) {
        this.i = i;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        i.spinIn();
        t.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return t.getElapsedTime() >= INTAKE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        i.spinIdle();
    }
}