package org.firstinspires.ftc.teamcode.opmodes.subsystemtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@TeleOp
public class ShooterGateTest extends OpMode {
    public ShooterGate shooterGate;
    public DcMotor testMotor;

    public void init(){
        testMotor = hardwareMap.get(DcMotor.class, "shooterDriveMotor");
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterGate = new ShooterGate();
        shooterGate.init(hardwareMap);
    }

    public void loop(){
        testMotor.setPower(-gamepad1.left_stick_y);
        if(gamepad1.crossWasPressed()){
            shooterGate.open();
        } else if (gamepad1.circleWasPressed()){
            shooterGate.close();
        }

        shooterGate.updateLoop();
        telemetry.addData("GateState:" , shooterGate.getGateState());
    }
}
