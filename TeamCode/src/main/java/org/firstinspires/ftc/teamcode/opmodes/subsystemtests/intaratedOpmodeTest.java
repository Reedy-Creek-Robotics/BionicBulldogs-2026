package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "integratedOpcodeTest")

public class intaratedOpmodeTest extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    public Shooter shooter;
    public double velocityGoal;


    public void init() {
        //drivetrain init
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        //intake init
        intake = new Intake();
        intake.init(hardwareMap);
        //shooter init
        shooter = new Shooter();
        shooter.initialize(hardwareMap);
        velocityGoal = 0;
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
        //shooter
        if(gamepad1.dpadUpWasPressed()) {
            velocityGoal = velocityGoal + 0.1;
        }
        if(gamepad1.dpadDownWasPressed()){
            velocityGoal = velocityGoal - 0.1;
        }

        shooter.setVel(velocityGoal * 1500);
        telemetry.addData("power:", velocityGoal);
        telemetry.update();
    }
}
