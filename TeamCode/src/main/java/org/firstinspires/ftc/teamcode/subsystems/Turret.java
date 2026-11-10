package org.firstinspires.ftc.teamcode.subsystems;


import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    //CRServo turretServoOne;
    //CRServo turretServoTwo;
    DcMotorEx turretEncoder;

    public double K_p = 1.0;
    private static final int acceptableAngleError = 5;
    private static final int fullPowerThreshold = 500;
    public double angleGoal = 0;
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

    public double aimFieldRelative(Pose botPose, Pose aimGoal){
        double botRelativeAngle = findAngle(botPose, aimGoal);
        double fieldRelativeAngle = botRelativeAngle - botPose.heading();

        // Normalizing to -pi, pi
        fieldRelativeAngle = Math.atan2(
                Math.sin(fieldRelativeAngle),
                Math.cos(fieldRelativeAngle)
        );

        angleGoal = Math.toDegrees(-fieldRelativeAngle);
        return botRelativeAngle;
    }

    public Double findAngle(Pose botPose, Pose aimGoal){
        double botRelativeX = aimGoal.x() - botPose.x();
        double botRelativeY = aimGoal.y() - botPose.y();
        double aimAngle = Math.atan2(botRelativeY, botRelativeX);
        return aimAngle;
    }

    public void updateTurret() {
        double error = (int) Math.round(angleGoal * encoderTicksPerDegree) - turretEncoder.getCurrentPosition();
        if ((acceptableAngleError * encoderTicksPerDegree) < Math.abs(error)) {
            double power = error * K_p;
            if(Math.abs(power) > fullPowerThreshold){
                power = Math.abs(power) / power;
            } else {
                power = power / fullPowerThreshold;
            }
            moveServos(power);
        } else {moveServos(0);}
    }

    public void setKp(double newKp) {
        K_p = newKp;
    }
}
