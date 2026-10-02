package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;



@TeleOp(name = "TurretTestRedWithDrive")
public class TurretTestRedWithDrive extends OpMode{
    public Follower follower;
    public Turret turret;
    public Drivetrain drivetrain;
    Pose testPose;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7.75, 7.75, 0));
        turret = new Turret();
        turret.initialize(hardwareMap ,Turret.teamColor.redTeam, follower);
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        testPose = new Pose(72,72);
    }

    public void loop(){
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.driveFieldRelative(leftStickX, leftStickY, rightStickX);

        double aimAngle = turret.findAngleToPoint(testPose);
        double aimTicks = turret.moveToAngle(aimAngle);
        //double aimTicks = turret.updateLoop();
        telemetry.addData("State:", turret.getTurretState());
        telemetry.addData("Bot X:", follower.getPose().getX());
        telemetry.addData("Bot Y:", follower.getPose().getY());
        telemetry.addData("Bot Heading:", Math.toDegrees(follower.getPose().getHeading()));
        //telemetry.addData("Quad additive:", turret.findQuadrantAdditive(follower.getPose(), turret.pickAimPose()));
        telemetry.addData("Aim angle:", Math.toDegrees(turret.findAngleToPoint(testPose)));
        telemetry.addData("Tick Goal:", aimTicks);
        telemetry.addData("Turret Motor amps:", turret.turretDriveMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
        follower.update();
    }
}