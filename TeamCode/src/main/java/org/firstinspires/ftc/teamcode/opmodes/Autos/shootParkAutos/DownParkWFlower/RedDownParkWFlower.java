package org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos.DownParkWFlower;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos.GeneralShootParkAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRobber;

@Autonomous(name = "RedDownParkWFlower")
public class RedDownParkWFlower extends GeneralShootParkAuto {
    private static final int waitTime = 6;
    private ElapsedTime elapsedTime;

    public FlowerRobber flowerRobber;

    private final Pose start = poseFactory.of(FieldPoints.StartingPositions.redDown.x(), 144 - (17.25 / 2), 90);

    private final Pose parkZone = poseFactory.of(11.5755, 121.3112, 90);
    private final Pose parkControl1 = poseFactory.of(61.0076, 119.3683, 0);
    private final Pose frontOfFlower = poseFactory.of(48, 144 - 24, 90);
    private final Pose moveToFlowerControl = poseFactory.of(FieldPoints.StartingPositions.redDown.x(),144 - 48, 90);
    public Command autoRoutine() {
        return sequential(
                waitMs(1000),
                parallel(
                        follow(follower, MoveToFlowerFront()),
                        spinUpShooter,
                        sequential(
                                waitUntil(() -> follower.completion() > 0.1),
                                spinIntake,
                                waitUntil(() -> follower.completion() > 0.6),
                                lowerFlowerRobber
                        )
                ),
                waitMs((waitTime * 1000) - elapsedTime.milliseconds()),
                runShooterGate,
                follow(follower, CollectFromFlower()),
                stopShooter,
                stopIntake,
                parallel(
                        follow(follower, Park()),
                        sequential(
                                waitUntil(() -> follower.completion() > 0.4),
                                raiseFlowerRobber
                        )
                )
        );
    }

    @Override
    public void runOpMode() {
        initBaseVariables(start);

        flowerRobber = new FlowerRobber();
        flowerRobber.init(hardwareMap);

        elapsedTime = new ElapsedTime();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            updateTelemetry();
        }
    }

    public Path Park() {
        return Paths.curve(FieldPoints.FlowerIntakePose.scoringFlower, parkControl1, parkZone).constant(parkZone);
    }
    public Path MoveToFlowerFront(){
        return Paths.curve(start, moveToFlowerControl, frontOfFlower).constant(start);
    }
    public Path CollectFromFlower(){
        return Paths.line(frontOfFlower, FieldPoints.FlowerIntakePose.scoringFlower).constant(frontOfFlower);
    }
    public Command lowerFlowerRobber = Command.build()
            .setStart(() -> flowerRobber.setDOWN())
            .setDone(() -> true);
    public Command raiseFlowerRobber = Command.build()
            .setStart(() -> flowerRobber.setUP())
            .setDone(() -> true);
}
