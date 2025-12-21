package org.firstinspires.ftc.teamcode.decode_teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.util.Timer;
@TeleOp
public class testFile extends OpMode {
    Robot r;
    GamepadEx operatorGamepad;

    @Override
    public void init() {
        r = new Robot(hardwareMap, telemetry, Alliance.BLUE);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    public void start(){

    }

    @Override
    public void loop() {
        operatorGamepad.readButtons();

        if (gamepad2.a) {
            r.s.activate = true;
            r.s.forDistance(50);
        }
        if (gamepad2.x) {
            r.s.stopMotor();
        }

        if (gamepad2.dpadUpWasPressed()) {
            r.s.kickUp();
        }
        if (gamepad2.dpadDownWasPressed()){
            r.s.kickDown();
        }

        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            r.s.feedUp();
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.s.feedDown();
        }
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            r.s.feedZero();
        }

        telemetry.addData("Activate State", r.s.activate);
        telemetry.addData("Target Velocity", r.s.getTarget());
        telemetry.addData("Actual Velocity", r.s.getVelocity());
        telemetry.addLine("A: Spin Shooter | B: Stop Shooter");
        telemetry.addLine("X: Hold to Kick");
        telemetry.addLine("DPAD Up/Down/Left: Hood Positions");
        telemetry.update();
    }

}

