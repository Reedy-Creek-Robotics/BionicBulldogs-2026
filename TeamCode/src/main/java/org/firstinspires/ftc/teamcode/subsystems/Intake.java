package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    /*
    int IntakeState = 2;
    int FORWARD = 1;
    int STOP = 2;
    int BACKWARD = 3;
    */
    public enum possibleIntakeStates {
        FORWARD,
        STOP,
        BACKWARD
    }

    possibleIntakeStates IntakeState = possibleIntakeStates.STOP;
    public DcMotor intakeMotor;
    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotor.class,"intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0.0);

    }

    public possibleIntakeStates getIntakeState(){
        return IntakeState;
    }

    public void setFORWARD() {
            intakeMotor.setPower(1.0);
            IntakeState = possibleIntakeStates.FORWARD;
    }

    public void setBACKWARD() {
            intakeMotor.setPower(-1.0);
            IntakeState = possibleIntakeStates.BACKWARD;
    }

    public void setSTOP() {
            intakeMotor.setPower(0.0);
            IntakeState = possibleIntakeStates.STOP;
    }
    public void ToggleForward(){
        if (IntakeState == possibleIntakeStates.FORWARD){
            setSTOP();
        }
        else{
            setFORWARD();
        }
    }
    public void ToggleBackward(){
        if (IntakeState == possibleIntakeStates.BACKWARD){
            setSTOP();
        }
        else {
            setBACKWARD();
        }
    }

}
