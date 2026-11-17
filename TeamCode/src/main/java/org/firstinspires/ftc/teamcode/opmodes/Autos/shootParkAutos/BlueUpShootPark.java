package org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos;

import static com.pedropathing.ivy.Scheduler.schedule;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@Autonomous(name = "BlueUpShootPark", group = "Autonomous")
public class BlueUpShootPark extends GeneralShootParkAuto {

    private final Pose start = FieldPoints.StartingPositions.blueUp;
    private final Pose parkZone = poseFactory.of(129.0965, 48.0, -90);
    private final Pose parkControl1 = poseFactory.of(115.3758, 91.1158, 0);

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
