package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

public class FieldComponentsPose {
    public double tile = 23;
    public List<Pose> blueHivesPose = List.of(new Pose (59.25,72 - (Math.sin(30) * 15.44)), new Pose (59.25,72 + (Math.sin(30) * 15.44)));
    public List<Pose> redHivesPose = List.of(new Pose (84.75,72 - (Math.sin(30) * 15.44)), new Pose (84.75,72 + (Math.sin(30) * 15.44)));
    public List<Pose> flowers = List.of(new Pose(144 - (tile * 2), 0), new Pose(144, 144 - (tile * 2)), new Pose((tile * 2), 144), new Pose(0, (tile * 2))); // TODO: add offset from wall

    public List<Pose> getHivesPose(Turret.teamColor teamSet){
        if(teamSet == Turret.teamColor.blueTeam){
            return blueHivesPose;
        } else {
            return redHivesPose;
        }
    }

    public List<Pose> getFlowersPose(){
        return flowers;
    }
}
