package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="IntakeTest")
public class IntakeTest extends OpMode {

    double avgCurrent;
    double maxCurrent;

    Intake intake;

    @Override
    public void init() {
        intake = new Intake();
        intake.init(hardwareMap);
        telemetry.addLine("Initialized");
    }


    @Override
    public void loop() {
        if(gamepad1.rightBumperWasPressed()){
            intake.ToggleForward();
        }
        if(gamepad1.leftBumperWasPressed() ){
            intake.ToggleBackward();
        }
        if (gamepad1.x) {
            avgCurrent = intake.getAvgCurrent();
            maxCurrent = intake.getMaxCurrent();
        }

        telemetry.addData("Intake Current", intake.getMotorCurrent());
        telemetry.addData("Avg Current", avgCurrent);
        telemetry.addData("Max Current", maxCurrent);
        telemetry.update();
    }
}