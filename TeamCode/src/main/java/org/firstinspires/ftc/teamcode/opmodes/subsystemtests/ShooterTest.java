package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode{
    public Shooter shooter;
    public double powerGoal;

    public void init() {
        shooter = new Shooter();
        shooter.initialize(hardwareMap);
        powerGoal = 0;
    }

    public void loop() {
        double requestedPower = -gamepad1.left_stick_y;
        if(requestedPower > powerGoal) {
            powerGoal = requestedPower;
        }
        if(gamepad1.a){
            powerGoal = 0;
        }

        shooter.setPower(powerGoal);
        telemetry.addData("A:", gamepad1.a);
        telemetry.addData("power:", powerGoal);
        telemetry.addData("joystick:", powerGoal);
        telemetry.update();
    }
}
