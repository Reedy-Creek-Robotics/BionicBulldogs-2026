package org.firstinspires.ftc.teamcode.opmodes.soloAutos;

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
import org.firstinspires.ftc.teamcode.subsystems.FlowerRobber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

@Autonomous(name = "RedUpSoloFlowerAndGarden", group = "Autonomous")
public class RedUpSoloFlowerAndGarden extends LinearOpMode {


    private Follower follower;
    private Intake intake;
    private ShooterGate shooterGate;
    private Shooter shooter;
    private FlowerRobber flowerRobber;
    private static final int shooterVelocityGoal = 1800;
    private static final int acceptableShooterVelocityError = 50;

    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(59.1367, 5.2013, 90);
    private final Pose path1 = poseFactory.of(17.2735, 43.6762, 165);
    private final Pose path1Control1 = poseFactory.of(47, 36.7475, 0);
    private final Pose point2Start = poseFactory.of(17.2735, 43.6762, 165);
    private final Pose point2 = poseFactory.of(40.1292, 109.0805, 190);
    private final Pose point2Control1 = poseFactory.of(43.4471, 46.1062, 0);
    private final Pose point2Control2 = poseFactory.of(21.8406, 109.13, 0);
    private final Pose point3Start = poseFactory.of(40.1292, 109.0805, 190);
    private final Pose gardenIntake = poseFactory.of(8.4581, 9.8406, -90);
    private final Pose point3Control1 = poseFactory.of(6.4664, 97.2735, 0);
    private final Pose point3Control2 = poseFactory.of(33.5034, 31.0067, 0);
    private final Pose parkPose = poseFactory.of(10.6535, 95.9362, -90);
    private final Pose GardenToParkControl1 = poseFactory.of(27.8758, 45.9354, 0);

    public Path startToFlower() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path FlowerToDown() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    public Path DownToGarden() {
        return Paths.curve(point3Start, point3Control1, point3Control2, gardenIntake).linear(point3Start, gardenIntake);
    }

    public Path GardenToPark() {
        return Paths.curve(gardenIntake, GardenToParkControl1, parkPose).constant(parkPose);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                spinUpShooter,
                spinIntake,
                runShooterGate,
                lowerRober,
                follow(follower, startToFlower())
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

        flowerRobber = new FlowerRobber();
        flowerRobber.init(hardwareMap);

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
    Command lowerRober = Command.build()
            .setStart(() -> flowerRobber.setDOWN())
            .setDone(() -> true);
}
