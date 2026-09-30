package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode{
    public Shooter shooter;
    public double velocityGoal;

    public void init() {
        shooter = new Shooter();
        shooter.initialize(hardwareMap);
        velocityGoal = 0;
    }

    public void loop() {
        if(gamepad1.dpadUpWasPressed()) {
            velocityGoal = velocityGoal + 150;
        }
        if(gamepad1.dpadDownWasPressed()){
            velocityGoal = velocityGoal - 150;
        }

        shooter.setVel(velocityGoal);
        telemetry.addData("requested velocity:", velocityGoal);
        telemetry.addData("current velocity", shooter.getMotorVelocity());
        telemetry.update();
    }
}
