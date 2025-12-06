package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeIn extends CommandBase {
    private final Intake i;
    private Follower follower;
    private int st = 0;
    private Timer t = new Timer();
    public IntakeIn(Intake i) {
        this.i = i;
    }

    @Override
    public void initialize(){
        setState(0);
    }

    @Override
    public void execute(){
        switch (st){
            case 0:
                i.spinIn();
                setState(1);
                break;
            case 1:
                if (t.getElapsedTime() > 1500 && !follower.isBusy()) {
                    setState(-1);
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return st == -1;
    }



    private void setState(int i) {
        st = i;
        t.resetTimer();
    }
}
