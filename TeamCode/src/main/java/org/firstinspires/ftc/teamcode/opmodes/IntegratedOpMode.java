package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRobber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@TeleOp(name = "IntegratedOpMode")

public class IntegratedOpMode extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    public Shooter shooter;
    public double velocityGoal;
    public ShooterGate shooterGate;
    public FlowerRobber flowerRobber;
    public double blackboardHeading;

    //public DcMotor turretMotor;


    public void init() {
        //drivetrain init
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        //if(blackboard.containsKey("Heading")) {
        //    blackboardHeading = (double) blackboard.get("Heading");
        //    drivetrain.setPinpointHeading(blackboardHeading);
        //}

        //intake init
        intake = new Intake();
        intake.init(hardwareMap);

        //shooter init
        shooter = new Shooter();
        shooter.initialize(hardwareMap);
        velocityGoal = 1500;

        // Gate init
        shooterGate = new ShooterGate();
        shooterGate.init(hardwareMap);

        flowerRobber = new FlowerRobber();
        flowerRobber.init(hardwareMap);

        //telemetry.addData("blackboardHeading: ", blackboardHeading);
        telemetry.addLine("initialized");
        telemetry.update();

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
            velocityGoal = velocityGoal + 50;
            shooter.setVel(velocityGoal);
        }

        if(gamepad1.dpadDownWasPressed()){
            velocityGoal = velocityGoal - 50;
            shooter.setVel(velocityGoal);
        }

        //shooter on/off
        if (gamepad1.squareWasPressed()){
            shooter.setVel(velocityGoal);
        }
        if(gamepad1.triangleWasPressed()){
            shooter.setVel(0);
        }

        // Gate
        if(gamepad1.crossWasPressed()){
            shooterGate.open();
        }
        shooterGate.updateLoop();

        if(gamepad1.optionsWasPressed()){
            drivetrain.resetPinpoint();
        }

        if(gamepad1.circleWasPressed()){
            flowerRobber.toggleState();
        }

        ShooterGate.GateStates gateState = shooterGate.getGateState();

        // Telemetry
        telemetry.addData("Gate State: " , gateState);
        telemetry.addData("Intake State: ", intake.getIntakeState());
        telemetry.addData("Requested Shooter Vel: ", velocityGoal);
        telemetry.addData("Actual Shooter Vel: ", shooter.getMotorVelocity());
        telemetry.addData("Heading: ", drivetrain.getPinpointHeading());
        telemetry.addData("Flower Intake: ", flowerRobber.getState());
        telemetry.update();
    }
}
