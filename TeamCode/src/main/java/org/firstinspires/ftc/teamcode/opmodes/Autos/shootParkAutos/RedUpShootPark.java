package org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos;

import com.pedropathing.api.Paths;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;

@Autonomous(name = "RedUpShootPark", group = "Autonomous")
public class RedUpShootPark extends GeneralShootParkAuto {
    private final Pose start = FieldPoints.StartingPositions.redUp;
    private final Pose parkZone = poseFactory.of(10.7626, 92.943, 90);
    private final Pose parkControl1 = poseFactory.of(13.9992, 21.078, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                spinUpShooter,
                spinIntake,
                runShooterGate,
                stopShooter,
                stopIntake,
                follow(follower, Park())
        );
    }

    @Override
    public void runOpMode() {
        initBaseVariables(start);

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
}
