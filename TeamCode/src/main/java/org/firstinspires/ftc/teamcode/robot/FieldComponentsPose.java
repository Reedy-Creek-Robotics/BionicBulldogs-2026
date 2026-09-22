package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

public class FieldComponentsPose {
    public List<Pose> blueHivesPose = List.of(new Pose (59.25,72 - (Math.sin(30) * 15.44)), new Pose (59.25,72 + (Math.sin(30) * 15.44))); // TODO: set actual positions
    public List<Pose> redHivesPose = List.of(new Pose (84.75,72 - (Math.sin(30) * 15.44)), new Pose (84.75,72 + (Math.sin(30) * 15.44))); // TODO: set actual positions

    public List<Pose> getHivesPose(Turret.teamColor teamSet){
        if(teamSet == Turret.teamColor.blueTeam){
            return blueHivesPose;
        } else {
            return redHivesPose;
        }
    }


}
