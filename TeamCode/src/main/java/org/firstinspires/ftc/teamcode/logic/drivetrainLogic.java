package org.firstinspires.ftc.teamcode.logic;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class drivetrainLogic {
    public static double[] drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        double frontLeftMotorPower = maxSpeed * (frontLeftPower / maxPower);
        double backLeftMotorPower = maxSpeed * (backLeftPower / maxPower);
        double frontRightMotorPower = maxSpeed * (frontRightPower / maxPower);
        double backRightMotorPower = maxSpeed * (backRightPower / maxPower);

        return new double[]{frontLeftMotorPower, backLeftMotorPower, frontRightMotorPower, backRightMotorPower};
    }

    public static double[] driveFieldRelative(IMU imu, double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        return drivetrainLogic.drive(newForward, newStrafe, rotate);
    }
}
