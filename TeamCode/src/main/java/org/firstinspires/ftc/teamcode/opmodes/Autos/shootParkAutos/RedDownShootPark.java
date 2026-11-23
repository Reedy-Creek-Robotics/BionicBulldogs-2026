package org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;

@Autonomous(name = "RedDownShootPark", group = "Autonomous")
public class RedDownShootPark extends GeneralShootParkAuto {
    private static final int waitTime = 6;
    private ElapsedTime elapsedTime;

    private final Pose start = FieldPoints.StartingPositions.redDown;
    private final Pose parkZone = poseFactory.of(11.5755, 121.3112, -90);
    private final Pose parkControl1 = poseFactory.of(61.0076, 119.3683, 0);


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
}
