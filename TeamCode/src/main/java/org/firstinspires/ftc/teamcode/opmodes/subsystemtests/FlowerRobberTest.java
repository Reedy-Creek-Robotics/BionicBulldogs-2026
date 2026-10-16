package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlowerRobber;


@TeleOp(name="flowerRobberTest")
public class FlowerRobberTest extends OpMode {
    FlowerRobber flowerRobber;

    @Override
    public void init() {
        flowerRobber = new FlowerRobber();
        flowerRobber.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.circleWasPressed()){
            flowerRobber.toggleState();
        }
        telemetry.addData("Fliper State: ", flowerRobber.getState());
        telemetry.update();
    }
}
