package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;

@TeleOp(name="AprilTagTest")
public class AprilTagTest extends OpMode {
    AprilTag apriltagSub = new AprilTag();
    AprilTag.Hive hiveState = AprilTag.Hive.None;


    @Override
    public void init() {
        apriltagSub.init(AprilTag.Teams.RED, hardwareMap, telemetry);
    }


    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()){
            apriltagSub.useCamOn();
        }
        if (gamepad1.circleWasPressed()){
            apriltagSub.useCamOff();
        }
        apriltagSub.TeamSetRun();
    }
}