package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;



@TeleOp(name = "IntakeDrivetrain")
public class IntakeDrivetrain extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    public void init() {
        //drivetrain
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        //intake
        intake = new Intake();
        intake.init(hardwareMap);
    }

    public void loop() {
        //drivetrain
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.driveFieldRelative(leftStickX, leftStickY, rightStickX);
        //intake
        if(gamepad1.rightBumperWasPressed()){
            intake.ToggleForward();
        }
        if(gamepad1.leftBumperWasPressed() ){
            intake.ToggleBackward();
        }
    }


}
