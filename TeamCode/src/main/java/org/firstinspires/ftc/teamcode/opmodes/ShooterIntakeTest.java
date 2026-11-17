package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
    @TeleOp(name = "ShooterIntakeTest")

    public class ShooterIntakeTest extends OpMode {
        Intake intake;
        public Shooter shooter;
        public double velocityGoal;


        public void init() {
            //intake init
            intake = new Intake();
            intake.init(hardwareMap);

            //shooter init
            shooter = new Shooter();
            shooter.initialize(hardwareMap);
            velocityGoal = -1500;
            telemetry.addLine("initialized");
            telemetry.update();
        }

        public void loop() {
            //intake
            if(gamepad1.rightBumperWasPressed()){
                intake.ToggleForward();
            }
            if(gamepad1.leftBumperWasPressed() ){
                intake.ToggleBackward();
            }

            //shooter
            if(gamepad1.dpadUpWasPressed()) {
                velocityGoal = velocityGoal + 50;
                shooter.setVel(velocityGoal);
            }

            if(gamepad1.dpadDownWasPressed()){
                velocityGoal = velocityGoal - 50;
                shooter.setVel(velocityGoal);
            }

            //shooter on/off
            if (gamepad1.squareWasPressed()){
                shooter.setVel(velocityGoal);
            }
            if(gamepad1.triangleWasPressed()){
                shooter.setVel(0);
            }

            telemetry.addData("intake state:", intake.getIntakeState());
            telemetry.addData("requested vel:", velocityGoal);
            telemetry.addData("actual vel:", shooter.getMotorVelocity());
            telemetry.update();
        }
    }
