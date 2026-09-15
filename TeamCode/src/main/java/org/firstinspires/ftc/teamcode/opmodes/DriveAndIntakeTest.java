package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "DriveTrainTest")
public class DriveAndIntakeTest extends OpMode {
    Drivetrain drivetrain;
    Intake intake;

    public void init() {
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);
    }

    public void loop() {
        processDriving();
        processIntake();
    }

    protected void processDriving() {
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.driveFieldRelative(leftStickY, leftStickX, rightStickX);
    }

    protected void processIntake() {
        if( gamepad1.rightBumperWasPressed() ) {
            intake.toggleForward();
        }

        if( gamepad1.leftBumperWasPressed() ) {
            intake.toggleReverse();
        }
    }
}
