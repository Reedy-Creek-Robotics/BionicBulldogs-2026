package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="IntakeTester")
public class IntakeTester extends OpMode {

    Intake intake;

    @Override
    public void init() {
        intake = new Intake();
        intake.init(hardwareMap);
        telemetry.addLine("Initialized");
    }

    double totalCurrent = 0;
    double counter = 0;
    double avgCurrent = 0;
    @Override
    public void loop() {
        if(gamepad1.rightBumperWasPressed()){
            intake.ToggleForward();
        }
        if(gamepad1.leftBumperWasPressed() ){
            intake.ToggleBackward();
        }

        counter = counter + 1;
        if (intake.IntakeState != 2) {
            if (counter % 10 == 0) {
                totalCurrent = totalCurrent + intake.getMotorCurrent();
                avgCurrent = totalCurrent / (counter / 10);
            }
        }

        telemetry.addData("Intake Current", intake.getMotorCurrent());
        telemetry.addData("Avg Current While On:", avgCurrent);
        telemetry.update();
    }
}

