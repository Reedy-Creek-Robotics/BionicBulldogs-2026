package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.logic.drivetrainLogic;
import org.firstinspires.ftc.teamcode.robot.drivetrainSetup;

public class drivetrain {
    public static void main(drivetrainSetup driveTrain, IMU imu, double forward, double strafe, double rotate){
        double[] drivePower = drivetrainLogic.driveFieldRelative(imu, forward, strafe, rotate);
        driveTrain.frontLeftMotor.setPower(drivePower[0]);
        driveTrain.backLeftMotor.setPower(drivePower[1]);
        driveTrain.frontRightMotor.setPower(drivePower[2]);
        driveTrain.backRightMotor.setPower(drivePower[3]);
    }
}
