package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    enum MotorState {FORWARD, REVERSE, STOPPED}
    MotorState currentState;

    DcMotorEx intakeMotor;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0.0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentState = MotorState.STOPPED;
    }

    public void toggleForward() {
        if( currentState == MotorState.FORWARD ) {
            stop();
        }
        else {
            forward();
        }
    }

    public void toggleReverse() {
        if( currentState == MotorState.REVERSE ) {
            stop();
        }
        else {
            reverse();
        }
    }

    protected void forward() {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setPower(1.0);
        currentState = MotorState.FORWARD;
    }

    protected void reverse() {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(-1.0);
        currentState = MotorState.REVERSE;
    }

    public void stop() {
        intakeMotor.setPower(0.0);
        currentState = MotorState.STOPPED;
    }
}
