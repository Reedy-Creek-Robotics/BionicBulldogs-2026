package org.firstinspires.ftc.teamcode.opmodes.shootParkAutos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

import java.util.Map;
import java.util.concurrent.TimeUnit;

@Autonomous
public class ShootParkRedUp extends LinearOpMode {
    private static final int velocityGoal = 903;
    private static final int shooterError = 20;
    private static final int timeToDrive = 500;
    private static final int escapeTime = 20;
    Shooter shooter;
    ShooterGate shooterGate;
    Drivetrain drivetrain;
    ElapsedTime timeTracker;
    Intake intake;

    public static final PoseFactory poseFactory = PoseFactory.degrees();

    public Pose startPose;
    public Pose parkingPose;
    public Follower follower;

    public Path startToPark(){
        //return Paths.curve(startPose, poseFactory.of(24,48, 0) , parkingPose).constant(startPose);
        return Paths.line(startPose, poseFactory.of(72,72,90)).constant(startPose);
    }
    public void driveForward(){
        intake.setSTOP();
        shooter.setVel(0);

        schedule(follow(follower, startToPark()));
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("x: ", follower.pose().x());
            telemetry.addData("y: ", follower.pose().y());
            telemetry.addData("Goal: ", parkingPose);
            Map<String, Object> drivetrainLog = follower.debug().drivetrain();
            Map<String, Object> algorithmLog = follower.debug().algorithm();
            Map<String, Object> localizerLog = follower.debug().localizer();
            Map<String, Object> followStateLog = follower.debug().followState();
            telemetry.addData("arivetrainLog: ", drivetrainLog);
            telemetry.addData("algorithmLog: ", algorithmLog);
            telemetry.addData("localizerLog: ", localizerLog);
            telemetry.addData("followStateLog: ", followStateLog);
            telemetry.update();
            Scheduler.execute();
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

        follower = Constants.create(hardwareMap);
        follower.setPose(FieldPoints.StartingPositions.redUp);
        follower.update();

        Scheduler.reset();

        startPose = FieldPoints.StartingPositions.redUp;
        parkingPose = FieldPoints.ParkingZones.redUpPark;

        // ===========================================================================================================================================

        waitForStart();

        // ===========================================================================================================================================

        telemetry.addData("Reached Velocity: ", false);
        telemetry.addData("Velocity: ", shooter.getMotorVelocity());
        telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
        telemetry.addData("Time Return: ", TimeDrive());
        telemetry.addData("Gate State: ", shooterGate.getGateState());
        telemetry.addData("x: ", follower.pose().x());
        telemetry.addData("y: ", follower.pose().y());
        telemetry.update();

        // init
        // spin up shooter
        // start intake
        // open gate
        // call move

        // Spin up shooter
        shooter.setVel(velocityGoal);

        // Wait for shooter to reach vel goal
        while (Math.abs(velocityGoal - shooter.getMotorVelocity()) > shooterError){
            // I am an empty loop :D
            boolean timeReturn = TimeDrive();
            telemetry.addData("Reached Velocity: ", false);
            telemetry.addData("Velocity: ", shooter.getMotorVelocity());
            telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
            telemetry.addData("Time Return: ", timeReturn);
            telemetry.addData("Gate State: ", shooterGate.getGateState());
            telemetry.addData("x: ", follower.pose().x());
            telemetry.addData("y: ", follower.pose().y());
            telemetry.update();
            if(timeReturn){
                return;
            }
            // not anymore D:
        }

        // Start intake/transfer
        intake.ToggleForward();

        // Opening gate
        shooterGate.open();

        telemetry.addData("Reached Velocity: ", true);
        telemetry.addData("Velocity: ", shooter.getMotorVelocity());
        telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
        telemetry.addData("Time Return: ", TimeDrive());
        telemetry.addData("Gate State: ", shooterGate.getGateState());
        telemetry.addData("x: ", follower.pose().x());
        telemetry.addData("y: ", follower.pose().y());
        telemetry.update();

        // Wait for shooter to spin up
        while (shooterGate.getGateState() != ShooterGate.GateStates.inactive){
            shooterGate.updateLoop();
            telemetry.addData("Reached Velocity: ", true);
            telemetry.addData("Velocity: ", shooter.getMotorVelocity());
            telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
            telemetry.addData("Time Return: ", TimeDrive());
            telemetry.addData("Gate State: ", shooterGate.getGateState());
            telemetry.addData("x: ", follower.pose().x());
            telemetry.addData("y: ", follower.pose().y());
            telemetry.update();
            if(TimeDrive()){
                return;
            }
        }

        // ================================================================================================
        // Moving forward
        intake.ToggleForward();
        driveForward();
        telemetry.addData("Reached Velocity: ", true);
        telemetry.addData("Velocity: ", shooter.getMotorVelocity());
        telemetry.addData("CurrentTime: ", timeTracker.time(TimeUnit.SECONDS));
        telemetry.addData("Time Return: ", TimeDrive());
        telemetry.addData("Gate State: ", shooterGate.getGateState());
        telemetry.addData("x: ", follower.pose().x());
        telemetry.addData("y: ", follower.pose().y());
        telemetry.update();
    }
}