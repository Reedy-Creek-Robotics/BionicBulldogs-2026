package org.firstinspires.ftc.teamcode.opmodes.soloAutos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
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


    private final Pose start = FieldPoints.StartingPositions.redUp;
    private final Pose flowerIntake = FieldPoints.FlowerIntakePose.leftFlower;
    private final Pose startToFlowerControl = poseFactory.of(32.75, 48.54, 0);
    private final Pose shootingPose = poseFactory.of(34.07, 131.52, 190);
    private final Pose shootingControl1 = poseFactory.of(43.45, 46.11, 0);
    private final Pose shootingControl2 = poseFactory.of(20.69, 105.85, 0);
    private final Pose shootToGarden = shootingPose;
    private final Pose gardenIntake = FieldPoints.GardenIntakePose.redGarden;
    private final Pose toGardenControl1 = poseFactory.of(6.4664, 97.2735, 0);
    private final Pose toGardenControl2 = poseFactory.of(25.36, 37.4, 0);
    private final Pose gardenWallHugStart = poseFactory.of(8.25, 31, 0);
    private final Pose gardenWallHugControl = poseFactory.of(5.62, 25.2, 0);
    private final Pose parkPose = FieldPoints.ParkingZones.redUpPark;
    private final Pose GardenToParkControl1 = poseFactory.of(27.8758, 45.9354, 0);

    public Path StartToFlower() {
        return Paths.curve(start, startToFlowerControl, flowerIntake).linear(start, flowerIntake);
    }

    public Path FlowerToDown() {
        return Paths.curve(flowerIntake, shootingControl1, shootingControl2, shootingPose).linear(flowerIntake, shootingPose);
    }

    public Path DownToGarden() {
        return Paths.curve(shootToGarden, toGardenControl1, toGardenControl2, gardenWallHugStart).linear(shootToGarden, gardenIntake);
    }
    public Path gardenWallHug() {
        return Paths.curve(gardenWallHugStart, gardenWallHugControl, gardenIntake).constant(gardenIntake);
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
                lowerRobber,
                follow(follower, StartToFlower()),
                waitMs(2000),
                parallel(
                        follow(follower, FlowerToDown()),
                        sequential(
                                waitMs(2000),
                                raiseRobber
                        )
                ),
                runShooterGate,
                follow(follower, DownToGarden()),
                follow(follower, gardenWallHug())
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
    Command lowerRobber = Command.build()
            .setStart(() -> flowerRobber.setDOWN())
            .setDone(() -> true);
    Command raiseRobber = Command.build()
            .setStart(() -> flowerRobber.setUP())
            .setDone(() -> true);
}
