package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.FieldComponentsPose;

import java.util.ArrayList;
import java.util.List;

public class Turret {
    public static final double gearRatio = 208.0 / 60.0;
    public DcMotor turretDriveMotor;
    public double currentAngle;
    private static final double TICKS_PER_MOTOR_REV = 1993.6;
    private static final double motorPower = 1;
    private Follower follower;
    public List<Pose> hivePositions = new ArrayList<Pose>();
    public FieldComponentsPose fieldComponents;


    // TODO: see if this should be put somewhere else for more general use
    public enum teamColor{
        redTeam,
        blueTeam
    }

    public enum turretState{
        moving,
        idle
    }

    public void initialize(HardwareMap hwMap, teamColor currentTeam, Follower passedFollower){
        fieldComponents = new FieldComponentsPose();
        follower = passedFollower;
        turretDriveMotor = hwMap.get(DcMotor.class, "TurretDriveMotor");
        turretDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretDriveMotor.setPower(0.0);
        turretDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentAngle = turretDriveMotor.getCurrentPosition();
        hivePositions = fieldComponents.getHivesPose(currentTeam);
    }

    private double findAngleToPoint(Pose goalPoint){
        Pose botPos = follower.getPose();
        double angleToAim = (Math.toDegrees(Math.atan((botPos.getY() - goalPoint.getX()) / (botPos.getX() - goalPoint.getX())))) - botPos.getHeading();
        return angleToAim;
    }

    private Pose pickAimPose(){
        if(follower.getPose().getY() > 72){
            return hivePositions.get(0);
        }
        return hivePositions.get(1);
    }

    private  void updateCurrentAngle(){
        currentAngle = turretDriveMotor.getCurrentPosition();
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

    public void updateLoop(){
        moveToAngle(findAngleToPoint(pickAimPose()));
    }
}