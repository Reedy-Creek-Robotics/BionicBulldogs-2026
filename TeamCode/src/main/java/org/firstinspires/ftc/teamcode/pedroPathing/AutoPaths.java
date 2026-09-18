package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Math.sqrt;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class AutoPaths extends OpMode{
    public static Follower follower;

    public final Pose point1A = new Pose (7.75, 7.75, 0);
    public final Pose point1B = new Pose(136.25, 7.75, 180);
    public final Pose point2 = new Pose(48, 72, 90);
    public final Pose point3 = new Pose(120, 24, 180);
    public final Pose point3_1 = new Pose(48, 120, 180);
    public final Pose control3_5 = new Pose(3.1-(24*sqrt(2)/2), 120-(24*sqrt(2)/2), 135);
    public final Pose point4 = new Pose(120, 120, 270);
    public final Pose control4_5 = new Pose(105, 96, 315);
    public final Pose point5 = new Pose(96, 72, 90);
    public final Pose control5_1 = new Pose(120, 96, 0);
    public final Pose control5_2 = new Pose(132, 72, 270);
    public final Pose control5_3 = new Pose(96, 48, 180);

    public Path startPath;
    public PathChain path2_5;

    public final Pose leftStart = new Pose(7.75, 7.75);
    public final Pose rightStart = new Pose(136.25, 7.75);

    public static void drawCurrent() {
        try {
            PanelsControl.drawRobot(follower.getPose());
            PanelsControl.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        PanelsControl.drawDebug(follower);
    }

    public void start() {
        follower.activateAllPIDFs();
        follower.followPath(startPath);
    }

    // used so the bot does not try to change paths after the main path is started
    public boolean reachedPoint2 = false;
    public void loop() {
        follower.update();
        drawCurrentAndHistory();

        // waiting for the bot to move to spot 2
        if(!reachedPoint2) {
            if(!follower.isBusy()){
                reachedPoint2 = true;
                follower.followPath(path2_5);
            }
        }
    }
}
