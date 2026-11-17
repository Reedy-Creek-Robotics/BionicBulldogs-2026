package org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@Autonomous(name = "BlueDownShootPark", group = "Autonomous")
public class BlueDownShootPark extends GeneralShootParkAuto {
    private static final int waitTime = 6;
    private ElapsedTime elapsedTime;

    private final Pose start = FieldPoints.StartingPositions.blueDown;
    //private final Pose parkZone = FieldPoints.ParkingZones.blueDownPark;
    private final Pose parkZone = poseFactory.of(128.5,20.5, 90);
    private final Pose parkControl1 = poseFactory.of(77.3, 41.6, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                spinUpShooter,
                spinIntake,
                waitMs((waitTime * 1000) - elapsedTime.milliseconds()),
                runShooterGate,
                stopShooter,
                stopIntake,
                follow(follower, Park())
                //follow(follower, Park1()),
                //waitMs(10000),
                //follow(follower, Park2())
        );
    }

    @Override
    public void runOpMode() {
        initBaseVariables(start);

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
        return Paths.curve(start, parkControl1, parkZone).constant(parkZone);
    }

    // public Path Park1() {
        //return Paths.line(start, parkControl1).constant(parkZone);
    //}
    // public Path Park2() {
        //return Paths.line(parkControl1, parkZone).constant(parkZone);
    //}
}
