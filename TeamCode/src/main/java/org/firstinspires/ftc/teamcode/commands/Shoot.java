package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWait;

public class Shoot extends CommandBase {
    Robot r;
    public Follower follower;
    private int st = 0;
    private Timer t = new Timer();

    public Shoot(Robot r) {
        this.r = r;
    }

    @Override
    public void initialize(){
        setState(0);
    }

    @Override
    public void execute(){
        switch (st){
            case 0:
                r.s.feedUp();
                r.s.setTarget(200);
                setState(1);
                break;
            case 1:
                if (r.s.isAtVelocity(200)){
                    setState(2);
                }
                break;
            case 2:
                if (t.getElapsedTime() > 0){
                    r.s.kickUp();
                }
                if (t.getElapsedTime() > 0.3){
                    r.s.kickDown();
                }
                if (t.getElapsedTime() > 0.6){
                    r.i.spinIn();
                }
                if (t.getElapsedTime() > 0.9){
                    r.i.spinIdle();
                    r.s.kickUp();
                }
                if (t.getElapsedTime() > 1.2){
                    r.s.kickDown();
                }
                if (t.getElapsedTime() > 1.5){
                    r.i.spinIn();
                }
                if (t.getElapsedTime() > 1.8){
                    r.i.spinIdle();
                    r.s.kickUp();
                }
                if (t.getElapsedTime() > 2.1){
                    r.s.kickDown();
                }
                break;
            case 3:
                if (t.getElapsedTime() > 2.15 && !follower.isBusy()){
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
