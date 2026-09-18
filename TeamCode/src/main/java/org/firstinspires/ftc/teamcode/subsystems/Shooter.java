package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotor shooterDriveMotor;

    public void initialize(HardwareMap hwMap){
        shooterDriveMotor = hwMap.get(DcMotor.class, "shooterDriveMotor");
        shooterDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterDriveMotor.setPower(0);
    }
    public void setPower(double requestedPower){
        shooterDriveMotor.setPower(requestedPower);
    }
}
