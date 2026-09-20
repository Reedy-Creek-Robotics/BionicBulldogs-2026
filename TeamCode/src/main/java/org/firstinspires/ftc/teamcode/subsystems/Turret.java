package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Turret {
    public static final double gearRatio = 208.0 / 60.0;
    public DcMotor turretDriveMotor;
    public double currentAngle;
    private static final double TICKS_PER_MOTOR_REV = 1993.6;
    private static final double motorPower = 1;
    private Follower follower;
    private hiveSet hivePositions;

    public record hiveSet(Pose upperHive, Pose lowerHive) {}

    // TODO: see if this should be put somewhere else for more general use
    public enum teamColor{
        redTeam,
        blueTeam
    }

    public void initialize(HardwareMap hwMap, teamColor currentTeam){
        follower = Constants.createFollower(hwMap); //TODO: move to opmode init
        turretDriveMotor = hwMap.get(DcMotor.class, "TurretDriveMotor");
        turretDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretDriveMotor.setPower(0.0);
        turretDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentAngle = turretDriveMotor.getCurrentPosition();
        if(currentTeam == teamColor.blueTeam){
            hivePositions = hiveSet(new Pose (30,30), new Pose (30,114)); // TODO: set actual positions 
        } else {
            hivePositions = hiveSet(new Pose (114,30), new Pose (114,114)); // TODO: set actual positions 
        }
    }

    public void updateAim(){
        moveToAngle(findAngleToPoint(pickAimPose()));
    }


    private double findAngleToPoint(Pose goalPoint){
        Pose botPos = follower.getPose();
        double angleToAim = (Math.toDegrees(Math.atan((botPos.getY() - goalPoint.getX()) / (botPos.getX() - goalPoint.getX())))) - botPos.getHeading();
        return angleToAim;
    }

    private Pose pickAimPose(){
        if(follower.getPose.gety > 72){
            return hivePositions.upperHive;
        }
        return hivePositions.lowerHive;
    }

    private  void updateCurrentAngle(){
        currentAngle = turretDriveMotor.getCurrentPosition();
    }

    public enum turretState{
        moving,
        idle
    }

    public turretState getUpdate(){
        if(turretDriveMotor.isBusy()){
            return turretState.moving;
        }
        return turretState.idle;
    }

    public void moveToAngle(double angleGoal){
        updateCurrentAngle();
        int targetTicks = (int) Math.round(((angleGoal / 360.0) - (currentAngle / TICKS_PER_MOTOR_REV)) * TICKS_PER_MOTOR_REV * gearRatio);
        turretDriveMotor.setTargetPosition(targetTicks);
        turretDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //TODO: see if this can be done in init
        turretDriveMotor.setPower(motorPower);
    }
}