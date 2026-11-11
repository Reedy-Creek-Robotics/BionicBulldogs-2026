package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Test")
public class TurretTest extends OpMode {
    public Turret turret;
    Follower follower;
    PoseFactory poseFactory;
    Pose goalPose;
    double angleGoal = 0;
    public final int goalAdjust = 20;
    public void init(){
        turret = new Turret();
        turret.init(hardwareMap, Turret.Team.RED);

        follower = Constants.create(hardwareMap);

        poseFactory = PoseFactory.degrees();

        follower.setPose(FieldPoints.StartingPositions.redUp);

        goalPose = poseFactory.of(0,0,0);
    }

    public void loop(){
        if(gamepad1.dpadUpWasPressed()){
            goalPose = poseFactory.of(goalPose.x(),goalPose.y() + goalAdjust,goalPose.heading());
            angleGoal = angleGoal + 5;
        } else if(gamepad1.dpadDownWasPressed()){
            goalPose = poseFactory.of(goalPose.x(),goalPose.y() - goalAdjust,goalPose.heading());
            angleGoal = angleGoal - 5;
        } else if(gamepad1.dpadRightWasPressed()){
            goalPose = poseFactory.of(goalPose.x() + goalAdjust,goalPose.y(),goalPose.heading());
            turret.setKp(turret.K_p + 0.01);
        } else if(gamepad1.dpadLeftWasPressed()){
            goalPose = poseFactory.of(goalPose.x() - goalAdjust,goalPose.y(),goalPose.heading());
            turret.setKp(turret.K_p - 0.01);
        }

        follower.update();

        turret.updateTurret(follower.pose());

        telemetry.addData("Bot X: ", follower.pose().x());
        telemetry.addData("Bot y: ", follower.pose().y());
        telemetry.addData("Bot Heading: ", follower.pose().heading());
        telemetry.addData("AimSide: ", turret.hiveSide);
        telemetry.addData("K_p: ", turret.K_p);
        telemetry.addData("Aim Goal:", turret.angleGoal);
        telemetry.update();
    }
}
