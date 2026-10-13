package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsControl;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@Autonomous
public class ShootParkAuto extends LinearOpMode {
    public enum TeamColor {
        RED,
        BLUE
    }

    public enum StartPositions {
        UP,
        DOWN
    }
    public TeamColor selectedTeamColor;
    public StartPositions selectedStartPositions;
    public Pose startPose;

    private static final int velocityGoal = 903;
    private static final int shooterVelocityError = 20;
    private static final int escapeTime = 23;
    Shooter shooter;
    ShooterGate shooterGate;
    Drivetrain drivetrain;
    Intake intake;
    Follower follower;

    ElapsedTime timeTracker;

    Pose parkPosition;

    public boolean checkApril(){
        return false;
    }

    @Override
    public void runOpMode() {
        // init
        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter();
        shooter.initialize(hardwareMap);

        shooterGate = new ShooterGate();
        shooterGate.init(hardwareMap);

        // Loop while the robot is in the initialization screen
        while ((selectedStartPositions == null) || (selectedTeamColor == null)) {
            if (gamepad1.dpadLeftWasPressed()) {
                selectedTeamColor = TeamColor.RED;
            } else if (gamepad1.dpadRightWasPressed()) {
                selectedTeamColor= TeamColor.BLUE;
            }

            if (gamepad1.dpadUpWasPressed()) {
                selectedStartPositions = StartPositions.UP;
            } else if (gamepad1.dpadDownWasPressed()) {
                selectedStartPositions = StartPositions.DOWN;
            }

            // Display live telemetry options
            telemetry.addData("=== AUTONOMOUS SELECTOR ===", "");
            telemetry.addData("Alliance (D-Pad L/R)", selectedTeamColor);
            telemetry.addData("Start Pos (D-Pad Up/Down)", selectedStartPositions);
            telemetry.update();

            if (selectedTeamColor == TeamColor.RED){
                if(selectedStartPositions == StartPositions.UP){
                    startPose = FieldPoints.StartingPositions.redUp;
                    parkPosition = FieldPoints.ParkingZones.redUpPark;
                } else {
                    startPose = FieldPoints.StartingPositions.redDown;
                    parkPosition = FieldPoints.ParkingZones.redDownPark;
                }
            } else if (selectedStartPositions == StartPositions.UP) {
                startPose = FieldPoints.StartingPositions.blueUp;
                parkPosition = FieldPoints.ParkingZones.blueUpPark;
            } else {
                startPose = FieldPoints.StartingPositions.blueDown;
                parkPosition = FieldPoints.ParkingZones.blueDownPark;
            }
        }

        waitForStart();

        follower.setStartingPose(startPose);
        follower.update();
        PanelsControl.drawDebug(follower);


        // Spin up turret
        // Wait for turret to spin up

        //if (selectedStartPositions == StartPositions.DOWN){
        //    while (!checkApril()){ // TODO: use James code
        //        // I'm empty become my maker is a bad programmer
        //    }
        //}

        // intake

        // Open gate
        // Wait for gate to finish cycle

        // Turn off shooter

        //follower.followPath(new Path(new BezierLine(startPose, parkPosition)));
        Path linePath = new Path(new BezierLine(startPose, new Pose(startPose.getX(), startPose.getY() + 24)));
        linePath.setConstantHeadingInterpolation(90);
        follower.followPath(linePath);
        follower.update();

        while (follower.isBusy()){
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("heading: ", follower.getPose().getHeading());
            telemetry.update();

            follower.update();
            PanelsControl.drawDebug(follower);
        }
    }
}
