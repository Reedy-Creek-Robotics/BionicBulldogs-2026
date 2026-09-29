package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotorEx shooterDriveMotor;

    public void initialize(HardwareMap hwMap){
        shooterDriveMotor = hwMap.get(DcMotorEx.class, "shooterDriveMotor");
        shooterDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterDriveMotor.setVelocity(0.0);
    }
    public void setVel(double requestedPower){
        shooterDriveMotor.setVelocity(requestedPower);
    }

}
