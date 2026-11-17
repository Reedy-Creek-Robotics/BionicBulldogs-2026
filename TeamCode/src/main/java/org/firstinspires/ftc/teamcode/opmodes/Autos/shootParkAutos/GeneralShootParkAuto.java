package org.firstinspires.ftc.teamcode.opmodes.Autos.shootParkAutos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;

public abstract class GeneralShootParkAuto extends LinearOpMode {

    public Follower follower;
    public Intake intake;
    public ShooterGate shooterGate;
    public Shooter shooter;

    public static final int shooterVelocityGoal = 1800;
    public static final int acceptableShooterVelocityError = 50;

    public final PoseFactory poseFactory = PoseFactory.degrees();
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

    public void initBaseVariables(Pose start) {
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
    }

    public void updateTelemetry(){
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
