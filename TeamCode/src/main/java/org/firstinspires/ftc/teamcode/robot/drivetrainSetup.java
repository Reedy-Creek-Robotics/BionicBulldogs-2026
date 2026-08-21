package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrainSetup {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public void init(HardwareMap hwMap){
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        DcMotor.ZeroPowerBehavior breakBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        frontLeftMotor.setZeroPowerBehavior(breakBehavior);
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setZeroPowerBehavior(breakBehavior);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setZeroPowerBehavior(breakBehavior);
        frontRightMotor.setPower(0.0);
        backRightMotor.setZeroPowerBehavior(breakBehavior);
        backRightMotor.setPower(0.0);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
