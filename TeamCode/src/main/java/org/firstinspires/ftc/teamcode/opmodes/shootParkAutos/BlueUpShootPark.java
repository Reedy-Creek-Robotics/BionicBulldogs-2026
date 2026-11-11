package org.firstinspires.ftc.teamcode.opmodes.shootParkAutos;

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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@Autonomous(name = "BlueUpShootPark", group = "Autonomous")
public class BlueUpShootPark extends LinearOpMode {


    private Follower follower;
    private Intake intake;
    private ShooterGate shooterGate;
    private Shooter shooter;
    private static final int shooterVelocityGoal = 1800;
    private static final int acceptableShooterVelocityError = 50;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84.5403, 131.9815, -90);
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
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        intake = new Intake();
        intake.init(hardwareMap);

        shooterGate = new ShooterGate();
        shooterGate.init(hardwareMap);

        shooter = new Shooter();
        shooter.initialize(hardwareMap);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Intake: ", intake.getIntakeState());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path Park() {
        return Paths.curve(start, parkControl1, parkZone).constant(parkZone);
    }

    Command spinIntake = Command.build()
            .setExecute(() -> intake.setFORWARD())
            .setDone(() -> intake.getIntakeState() == Intake.IntakeStates.FORWARD);

    Command stopIntake = Command.build()
            .setExecute(() -> intake.setSTOP())
            .setDone(() -> intake.getIntakeState() == Intake.IntakeStates.STOP);

    Command runShooterGate = Command.build()
            .setStart(() -> shooterGate.open())
            .setExecute(() -> shooterGate.updateLoop())
            .setDone(() -> shooterGate.getGateState() == ShooterGate.GateStates.inactive);

    Command spinUpShooter = Command.build()
            .setStart(() -> shooter.setVel(shooterVelocityGoal))
            .setDone(() -> Math.abs(shooter.getMotorVelocity() - shooterVelocityGoal) < acceptableShooterVelocityError);

    Command stopShooter = Command.build()
            .setStart(() -> shooter.setVel(0))
            .setDone(() -> true);
}
