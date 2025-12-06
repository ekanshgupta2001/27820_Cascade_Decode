package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Shoot extends CommandBase {
    public final Shooter shooter;
    public final Intake intake;
    public Follower follower;
    private int st = 0;
    private Timer t = new Timer();

    public Shoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void initialize(){
        setState(0);
    }

    @Override
    public void execute(){
        switch (st){
            case 0:
                shooter.scoreCloseSequence();
                setState(1);
                break;
            case 1:
                if (t.getElapsedTime() > 2000 && !follower.isBusy()){
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
