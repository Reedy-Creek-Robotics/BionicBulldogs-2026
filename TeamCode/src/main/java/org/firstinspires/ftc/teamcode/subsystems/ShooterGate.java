package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class ShooterGate {
    public enum GateStates {
        inactive,
        opening,
        closing,
        waiting
    }

    private GateStates currentState;
    private double timeGoal;
    private Servo gateServo;
    private static final double openTime = 1;
    private static final double closeTime = 1;
    private static final double waitTime = 1;
    private static final double openPos = 0;
    private static final double closePos = 0.3;
    private ElapsedTime gateTime;

    public void init(HardwareMap hwMap){
        gateServo = hwMap.get(Servo.class, "GateServo");
        timeGoal = 0.0;
        gateServo.setPosition(closePos);
        gateTime = new ElapsedTime();
        currentState = GateStates.inactive;
    }

    public void open(){
        gateServo.setPosition(openPos);
        gateTime.reset();
        timeGoal = openTime;
        currentState = GateStates.opening;
    }

    public void close(){
        gateServo.setPosition(closePos);
        gateTime.reset();
        timeGoal = closeTime;
        currentState = GateStates.closing;
    }

    public void waitOpen(){
        gateTime.reset();
        timeGoal = waitTime;
        currentState = GateStates.waiting;
    }

    public void updateLoop() {
        if (currentState == GateStates.inactive) {
            return;
        }
        if (gateTime.time(TimeUnit.SECONDS) < timeGoal) {
            return;
        }


        if (currentState == GateStates.opening) {
            waitOpen();
            return;
        }
        if (currentState == GateStates.waiting) {
            close();
            return;
        }
        if (currentState == GateStates.closing) {
            currentState = GateStates.inactive;
        }
    }

    public GateStates getGateState(){
        return currentState;
    }
}
