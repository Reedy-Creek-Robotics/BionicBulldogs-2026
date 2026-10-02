package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    /*
    int intakeState = 2;
    int FORWARD = 1;
    int STOP = 2;
    int BACKWARD = 3;
     */
    public enum IntakeStates {
        FORWARD,
        STOP,
        BACKWARD
    }
    IntakeStates intakeState = IntakeStates.STOP;
    double totalCurrent = 0;
    double counter = 0;
    double avgCurrent = 0;
    double maxCurrent = 0;


    public DcMotorEx intakeMotor;
    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0.0);
    }

    public IntakeStates getIntakeState(){
        return intakeState;
    }

    public void setFORWARD() {
        intakeMotor.setPower(1.0);
        intakeState = IntakeStates.FORWARD;
    }

    public void setBACKWARD() {
        intakeMotor.setPower(-1.0);
        intakeState = IntakeStates.BACKWARD;
    }

    public void setSTOP() {
        intakeMotor.setPower(0.0);
        intakeState = IntakeStates.STOP;    }
    public void ToggleForward(){
        if (intakeState == IntakeStates.FORWARD){
            setSTOP();
        }
        else{
            setFORWARD();
        }
    }
    public void ToggleBackward(){
        if (intakeState == IntakeStates.BACKWARD){
            setSTOP();
        }
        else {
            setBACKWARD();
        }
    }
    public double getMotorCurrent(){
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getAvgCurrent() {
        counter = counter + 1;
        if (intakeState != IntakeStates.STOP) {
            if (counter % 10 == 0) {
                totalCurrent = totalCurrent + getMotorCurrent();
                avgCurrent = totalCurrent / (counter / 10);
            }
        }
        return avgCurrent;
    }

    public double getMaxCurrent() {
        if (getMotorCurrent() > maxCurrent) {
            maxCurrent = getMotorCurrent();
        }
        return maxCurrent;
    }
}
