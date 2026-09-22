package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;



@TeleOp(name = "TurretTestRedWithDrive")
public class TurretTestRedWithDrive extends OpMode{
    public Follower follower;
    public Turret turret;
    public Drivetrain drivetrain;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        turret = new Turret();
        turret.initialize(hardwareMap ,Turret.teamColor.redTeam, follower);
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
    }

    public void loop(){
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.driveFieldRelative(leftStickX, leftStickY, rightStickX);

        turret.updateLoop();
        telemetry.addData("Turret:", turret.getUpdate());
        telemetry.update();
    }
}