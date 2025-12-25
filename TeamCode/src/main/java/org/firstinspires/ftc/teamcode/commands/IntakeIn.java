package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeIn extends CommandBase {
    private final Intake i;
    private final Follower follower;

    private int st = 0;
    private final Timer t = new Timer();

    public IntakeIn(Intake i, Follower follower) {
        // This command spins intake in for a short time, then ends when drive is done.
        this.i = i;
        this.follower = follower;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        // This starts the command fresh every time.
        setState(0);
    }

    @Override
    public void execute() {
        // This is a tiny state machine so intake timing is consistent.
        switch (st) {
            case 0:
                i.spinIn();
                setState(1);
                break;

            case 1:
                // Pedro Timer is in seconds, so 1.5 means 1.5 seconds.
                if (t.getElapsedTime() > 1.5 && !follower.isBusy()) {
                    setState(-1);
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // This ends when we hit the “done” state.
        return st == -1;
    }

    @Override
    public void end(boolean interrupted) {
        // This is just a safety stop so intake doesn’t keep running forever.
        i.spinIdle();
    }

    private void setState(int state) {
        // This changes state and resets the timer for the next phase.
        st = state;
        t.resetTimer();
    }
}