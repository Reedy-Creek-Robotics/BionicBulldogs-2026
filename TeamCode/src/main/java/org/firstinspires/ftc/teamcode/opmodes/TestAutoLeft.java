package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "AutoTestLeft", group = "Pedro Pathing")
public class TestAutoLeft extends AutoPaths {
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(point1A);
        follower.update();
        drawCurrent();
        path2_5 = follower.pathBuilder()
                .addPath(new BezierLine(point2, point3))
                .setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading())
                .addPath(new BezierLine(point3, point3_1))
                .setLinearHeadingInterpolation(point3.getHeading(),point3_1.getHeading())
                .build();
        startPath = new Path(new BezierLine(leftStart, point2));
    }
}
