package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TurretTestAuto")
public class TurretTestRed extends OpMode{
    public Turret turret;
    public void init(){
        turret = new Turret();
        turret.initialize(hardwareMap ,Turret.teamColor.redTeam);
    }

    public void loop(){
        turret.updateLoop();
        telemetry.addData("Turret:", turret.getUpdate());
        telemetry.update();
    }
}
