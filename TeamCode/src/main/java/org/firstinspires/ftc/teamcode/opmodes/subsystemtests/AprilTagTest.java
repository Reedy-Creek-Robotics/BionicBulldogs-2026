package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;

@TeleOp(name="AprilTagTest")
public class AprilTagTest extends OpMode {
    AprilTag apriltagSub = new AprilTag();
    AprilTag.Hive hiveState = AprilTag.Hive.None;
    double y = 70.75;


    @Override
    public void init() {
        apriltagSub.init(AprilTag.Teams.BLUE, hardwareMap, telemetry);
    }



    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()){
            y = 70.75;
        }
        if (gamepad1.triangleWasPressed()){
            y = 50.50;
        }
        telemetry.addData("y",y);
        apriltagSub.hiveGoal(y);
        telemetry.addData("hive" ,apriltagSub.hiveState);
        telemetry.update();
    }
}