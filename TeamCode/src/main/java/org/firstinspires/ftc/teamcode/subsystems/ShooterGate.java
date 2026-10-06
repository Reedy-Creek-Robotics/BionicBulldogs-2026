package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class ShooterGate {
    public enum gateStates{
        inactive,
        opening,
        closing,
        waiting
    }

    private gateStates currentState;
    private double timeGoal;
    private Servo gateServo;
    private static final double openTime = 1;
    private static final double closeTime = 1;
    private static final double waitTime = 1;
    private static final double openPos = 0;
    private static final double closePos = 0.5;
    private ElapsedTime gateTime;

    public void init(HardwareMap hwMap){
        gateServo = hwMap.get(Servo.class, "position_servo");
        timeGoal = 0.0;
        gateServo.setPosition(closePos);
        gateTime = new ElapsedTime();
    }

    public void open(){
        gateServo.setPosition(openPos);
        gateTime.reset();
        timeGoal = openTime;
        currentState = gateStates.opening;
    }

    public void close(){
        gateServo.setPosition(closePos);
        gateTime.reset();
        timeGoal = closeTime;
        currentState = gateStates.closing;
    }

    public void waitOpen(){
        gateTime.reset();
        timeGoal = waitTime;
        currentState = gateStates.waiting;
    }

    public gateStates updateLoop() {
        if (currentState == gateStates.inactive) {
            return currentState;
        }
        if (gateTime.time(TimeUnit.SECONDS) < timeGoal) {
            return currentState;
        }


        if (currentState == gateStates.opening) {
            waitOpen();
            return currentState;
        }
        if (currentState == gateStates.waiting) {
            close();
            return currentState;
        }
        if (currentState == gateStates.closing) {
            currentState = gateStates.inactive;
            return currentState;
        }
        return currentState;
    }
}
