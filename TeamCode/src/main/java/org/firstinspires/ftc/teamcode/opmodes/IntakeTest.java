package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="IntakeTest")
public class IntakeTest extends OpMode {

    Intake Intake;

    @Override
    public void init() {
        Intake = new Intake();
        Intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.rightBumperWasPressed()){
            Intake.ToggleForward();
        }
        if(gamepad1.leftBumperWasPressed() ){
            Intake.ToggleBackward();
        }
    }
}
