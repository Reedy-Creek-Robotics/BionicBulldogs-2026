package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.FieldComponentsPose;

import java.util.ArrayList;
import java.util.List;

public class Turret {
    public static final double gearRatio = 208.0 / 60.0;
    public DcMotorEx turretDriveMotor;
    public double currentMotorTicks;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double motorPower = 1;
    private Follower follower;
    private List<Pose> hivePositions = new ArrayList<Pose>();
    private final FieldComponentsPose fieldComponents = new FieldComponentsPose();
    private final int acceptableTickError = 10;


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
        follower = passedFollower;

        // Initializing the turret motor
        turretDriveMotor = hwMap.get(DcMotorEx.class, "TurretDriveMotor");
        turretDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretDriveMotor.setPower(0.0);
        turretDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretDriveMotor.setTargetPosition(turretDriveMotor.getCurrentPosition());
        turretDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretDriveMotor.setPower(motorPower);

        currentMotorTicks = turretDriveMotor.getCurrentPosition();
        hivePositions = fieldComponents.getHivesPose(currentTeam);
    }

    //public int findQuadrantAdditive(Pose bot, Pose goal){
    //    if(bot.getX() >= goal.getX() && bot.getY() >= goal.getY()){
    //        return 0;
    //    }
    //    if (bot.getX() < goal.getX() && bot.getY() >= goal.getY()) {
    //        return 90;
    //    }
    //    if (bot.getX() <= goal.getX() && bot.getY() < goal.getY()) {
    //        return 180;
    //    }
    //    return 270;
    //}

    //public double findAngleToPoint(Pose goalPoint){
    //    Pose botPos = follower.getPose();
    //    int quadrantAdditive = findQuadrantAdditive(botPos, goalPoint);
    //    double angleToAim = Math.abs(Math.toDegrees(Math.atan((botPos.getY() - goalPoint.getY()) / (botPos.getX() - goalPoint.getX())))) - Math.toDegrees(botPos.getHeading()) + quadrantAdditive;
    //    return angleToAim;
    //}

    // thx gpt
    public double findAngleToPoint(Pose goalPoint) {
        Pose botPos = follower.getPose();

        // Difference between target and robot in field coordinates
        double dx = goalPoint.getX() - botPos.getX();
        double dy = goalPoint.getY() - botPos.getY();

        // Absolute angle from the robot to the target
        double fieldAngle = Math.atan2(dy, dx);

        // Convert field angle to an angle relative to the robot
        double relativeAngle = fieldAngle - botPos.getHeading();

        // Normalize to [-PI, PI]
        relativeAngle = Math.atan2(
                Math.sin(relativeAngle),
                Math.cos(relativeAngle)
        );

        return Math.toDegrees(relativeAngle);
    }

    public Pose pickAimPose(){
        if(follower.getPose().getY() > 72){
            return hivePositions.get(0);
        }
        return hivePositions.get(1);
    }

    private  void updateCurrentAngle(){
        currentMotorTicks = turretDriveMotor.getCurrentPosition();
    }

    public turretState getTurretState(){
        if(turretDriveMotor.isBusy()){
            return turretState.moving;
        }
        return turretState.idle;
    }

    public int moveToAngle(double angleGoal){
        updateCurrentAngle();
        int targetTicks = (int) Math.round(((angleGoal / 360.0)) * TICKS_PER_MOTOR_REV * gearRatio);
        if (Math.abs(targetTicks - turretDriveMotor.getTargetPosition()) > acceptableTickError) {
            turretDriveMotor.setTargetPosition(targetTicks);
        }
        return targetTicks;
    }

    public int updateLoop(){
        double aimAngle = findAngleToPoint(pickAimPose());
        int tickGoal = moveToAngle(aimAngle);
        return tickGoal;
    }
}