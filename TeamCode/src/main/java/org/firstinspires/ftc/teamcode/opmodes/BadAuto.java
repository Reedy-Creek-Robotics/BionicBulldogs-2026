package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

import java.util.concurrent.TimeUnit;

@Autonomous
public class BadAuto extends LinearOpMode {
    private static final int velocityGoal = 903;
    private static final int shooterError = 20;
    private static final int timeToDrive = 500;
    private static final int escapeTime = 20;
    Shooter shooter;
    ShooterGate shooterGate;
    Drivetrain drivetrain;
    ElapsedTime timeTracker;
    Intake intake;

    public void driveForward(){
        timeTracker.reset();
        drivetrain.drive(0.5,0,0);
        while (timeTracker.time(TimeUnit.MILLISECONDS) < timeToDrive){
            // I'm also an empty loop :)
        }
    }

    public boolean TimeDrive(){
        if (timeTracker.time(TimeUnit.SECONDS) >= escapeTime){
            driveForward();
            return true;
        } else return false;
    }

    public void runOpMode(){
        // init
        shooter = new Shooter();
        shooter.initialize(hardwareMap);

        shooterGate = new ShooterGate();
        shooterGate.init(hardwareMap);

        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        timeTracker = new ElapsedTime();
        timeTracker.reset();
        waitForStart();

        telemetry.addData("Reached Velocity: ", false);
        telemetry.addData("Velocity: ", shooter.getMotorVelocity());
        telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
        telemetry.addData("Time Return: ", TimeDrive());
        telemetry.addData("Gate State: ", shooterGate.getGateState());
        telemetry.update();


        // Spin up shooter
        shooter.setVel(velocityGoal);

        while (Math.abs(velocityGoal - shooter.getMotorVelocity()) > shooterError){
            // I am an empty loop :D
            boolean timeReturn = TimeDrive();
            telemetry.addData("Reached Velocity: ", false);
            telemetry.addData("Velocity: ", shooter.getMotorVelocity());
            telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
            telemetry.addData("Time Return: ", timeReturn);
            telemetry.addData("Gate State: ", shooterGate.getGateState());
            telemetry.update();
            if(timeReturn){
                return;
            }
            // not anymore D:
        }

        intake.ToggleForward();

        // Opening gate
        shooterGate.open();

        telemetry.addData("Reached Velocity: ", true);
        telemetry.addData("Velocity: ", shooter.getMotorVelocity());
        telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
        telemetry.addData("Time Return: ", TimeDrive());
        telemetry.addData("Gate State: ", shooterGate.getGateState());
        telemetry.update();

        while (shooterGate.getGateState() != ShooterGate.GateStates.inactive){
            shooterGate.updateLoop();
            telemetry.addData("Reached Velocity: ", true);
            telemetry.addData("Velocity: ", shooter.getMotorVelocity());
            telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
            telemetry.addData("Time Return: ", TimeDrive());
            telemetry.addData("Gate State: ", shooterGate.getGateState());
            telemetry.update();
            if(TimeDrive()){
                return;
            }
        }

        // Moving forward
        intake.ToggleForward();
        driveForward();
        telemetry.addData("Reached Velocity: ", true);
        telemetry.addData("Velocity: ", shooter.getMotorVelocity());
        telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
        telemetry.addData("Time Return: ", TimeDrive());
        telemetry.addData("Gate State: ", shooterGate.getGateState());
        telemetry.update();

    }
}
