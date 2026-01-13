package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp( group = "A")
public class BumAhhTeleop extends OpMode {

    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    // private BumAhhShooter shooter;
    private Sensors sensors;
    private Follower follower;
    private GamepadEx player1, player2;

    private int shootStep = 0;
    private boolean autoAlignEnabled = true;
    private boolean isRedAlliance = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        drive = new MecanumDrive();
        drive.init(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        // shooter = new BumAhhShooter(hardwareMap);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        follower.update();
        sensors.update();
        player1.readButtons();
        player2.readButtons();

        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(-90)));
            gamepad1.rumble(500);
        }

        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            resetShooter();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shootStep++;
            if (shootStep > 2) {
                resetShooter();
            }
        }

        if (shootStep > 0) {
            double targetX = isRedAlliance ? 132.0 : 12.0;
            double targetY = 137.0;

//            shooter.setShooterTarget(
//                    follower.getPose().getX(), follower.getPose().getY(),
//                    targetX, targetY, 0, 0,
//                    Math.toDegrees(follower.getPose().getHeading()),
//                    autoAlignEnabled
//            );

            /* shooter.setTargetsByDistance(
                follower.getPose().getX(), follower.getPose().getY(),
                targetX, targetY,
                0.0
            );
            */

            if (shootStep == 1) {
                gamepad1.rumble(100);
            }

            if (shootStep == 2) {
                intake.transfer();
            }
        }

        if (shootStep == 0) {
            if (gamepad1.right_trigger > 0.2) intake.intake();
            else if (gamepad1.left_trigger > 0.2) intake.outtake();
            else intake.intakeMotorIdle();
        }

        if (player2.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        if (!autoAlignEnabled) {
            shooter.manualTurretOverride(-gamepad2.right_stick_x * 0.5, sensors.getSketchTurretPosition());
            // BumAhhShooter.targetTurretAngle = sensors.getSketchTurretPosition();
        }

        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = follower.getPose().getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);

        shooter.update(sensors.getFlywheelVelo(), sensors.getSketchTurretPosition());

        telemetry.addData("Step", shootStep);
        telemetry.addData("Align", autoAlignEnabled);
        telemetry.addData("Pose", follower.getPose().toString());
        telemetry.update();
    }

    private void resetShooter() {
        shootStep = 0;
        shooter.stopFlywheel();
        // shooter.stop();
        intake.intakeMotorIdle();
        gamepad1.stopRumble();
    }
}