package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class FlowerRobber {
    public enum RobberStates{
        up,
        down
    }
    RobberStates Robberstate;
    public Servo robber;
    public void init(HardwareMap hardwareMap){
        robber = hardwareMap.get(Servo.class, "robber");
        Robberstate = RobberStates.up;
        robber.setPosition(0.45);
    }

    public void setUP(){
        robber.setPosition(0.45);
        Robberstate = RobberStates.up;

    }
    public void setDOWN(){
        robber.setPosition(0.0);
        Robberstate = RobberStates.down;

    }
    public void toggleState(){
        if (Robberstate == RobberStates.up){
            setDOWN();
        } else{
            setUP();
        }
    }

    public RobberStates getState(){
        return Robberstate;
    }
}
