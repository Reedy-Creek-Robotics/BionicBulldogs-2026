package org.firstinspires.ftc.teamcode.opmodes.shootParkAutos;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@Autonomous()
public class ShootParkRedUp extends OpMode {
    public Pose startPose;
    public Pose parkingPose;
    public Follower follower;

    public Intake intake;
    public Shooter shooter;
    public ShooterGate shooterGate;

    @Override
    public void init() {
        startPose = FieldPoints.StartingPositions.redUp;
        parkingPose = FieldPoints.ParkingZones.redUpPark;

        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter();
        shooter.initialize(hardwareMap);

        shooterGate = new ShooterGate();
        shooterGate.init(hardwareMap);
    }

    public void loop(){

    }
}
