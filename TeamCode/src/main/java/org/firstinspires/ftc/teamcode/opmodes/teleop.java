package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.drivetrainSetup;
import org.firstinspires.ftc.teamcode.robot.imuSetup;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;

@TeleOp(name = "TeleOp")
public class teleop extends OpMode {
    imuSetup imu = new imuSetup();
    drivetrainSetup drivetrainMotors = new drivetrainSetup();
    public void init() {
        imu.init(HardwareMap);
    }

    public void loop() {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.main(drivetrainMotors, imu.imu, leftStickX, leftStickY, rightStickX);
    }
}
