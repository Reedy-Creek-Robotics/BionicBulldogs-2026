package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    //CRServo turretServoOne;
    //CRServo turretServoTwo;
    DcMotorEx turretEncoder;

    public double K_p = 0.0;
    private static final int acceptableAngleError = 5;
    double angleGoal = 0;
    private static final double encoderTicksPerDegree = 384.5 / 360.0; // NOT SET DO NOT USE OR YOU WILL SPEND YOUR LIFE DEBUGGING

    public void init(HardwareMap hardwareMap) {
//        turretServoOne = hardwareMap.get(CRServo.class, "turretServoOne");
//        turretServoTwo = hardwareMap.get(CRServo.class, "turretServoTwo");

        turretEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveServos(double power) {
//        turretServoOne.setPower(power);
//        turretServoTwo.setPower(power);
        turretEncoder.setPower(power);
    }

    public void aimBotRelative(double angle){
        angleGoal = angle;
    }

    public void updateTurret() {
        double error = (int) Math.round(angleGoal * encoderTicksPerDegree) - turretEncoder.getCurrentPosition();
        if (acceptableAngleError < Math.abs(error)) {
            moveServos(error * K_p);
        } else {moveServos(0);}
    }

    public void setKp(double newKp) {
        K_p = newKp;
    }
}
