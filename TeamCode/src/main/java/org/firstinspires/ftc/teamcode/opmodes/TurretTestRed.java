package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TurretTestRed")
public class TurretTestRed extends OpMode{
    public Follower follower;
    public Turret turret;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7.75, 7.75, 0));
        turret = new Turret();
        turret.initialize(hardwareMap ,Turret.teamColor.redTeam, follower);
    }

    public void loop(){
        telemetry.addData("Turret:", turret.getUpdate());
        telemetry.addData("Pose:", follower.getPose());
        Pose aimPose = turret.pickAimPose();
        telemetry.addData("Aim Goal", aimPose);
        double aimAngle = turret.findAngleToPoint(aimPose);
        telemetry.addData("Aim Angle:", aimAngle);
        if(gamepad1.aWasPressed()) {
            telemetry.addData("Tick goal:", turret.moveToAngle(aimAngle));
        }
        telemetry.update();
        follower.update();
    }
}
