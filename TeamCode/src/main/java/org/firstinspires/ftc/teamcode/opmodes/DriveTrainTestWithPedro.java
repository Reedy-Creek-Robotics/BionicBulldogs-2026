package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainWithPedro;

@TeleOp(name = "DriveTrainTestWithPedro")
public class DriveTrainTestWithPedro extends OpMode {

    public Follower follower;
    DrivetrainWithPedro drivetrain;
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7.75, 7.75, 0));
        drivetrain = new DrivetrainWithPedro();
        drivetrain.init(hardwareMap, follower);
    }

    public void loop() {
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.driveFieldRelative(leftStickX, leftStickY, rightStickX);
    }
}
