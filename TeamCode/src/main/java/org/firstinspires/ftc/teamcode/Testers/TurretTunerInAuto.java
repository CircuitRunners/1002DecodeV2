package org.firstinspires.ftc.teamcode.Testers;



import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous(name = "ONLY TURRET ALIGN", group = "Test")
public class TurretTunerInAuto extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Timer timer;

    // Field Constants (using your specific coordinates)
    private final double RED_GOAL_X = 127.0;
    private final double BLUE_GOAL_X = 17.0;
    private final double GOAL_Y = 127.5;

    public static double[] turretCoefficientsAuto = {0.087, 0.000, 0.00399995, 0.0009};


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry, true);
        timer = new Timer();

        telemetry.addLine("Wait for Alliance Selection...");
    }

    @Override
    public void init_loop() {
        // Use D-Pad Up for RED, D-Pad Down for BLUE
        Poses.updateAlliance(gamepad1, telemetry);

        telemetry.addData("Selected Alliance", Poses.getAlliance());
        telemetry.addData("Current Pose", follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        // Start the flywheel immediately

    }

    @Override
    public void loop() {

        shooter.turretCoefficientsAuto = turretCoefficientsAuto;
        follower.update();
        Pose currentPose = follower.getPose();

        // Determine target based on alliance
        double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

        // THE ALIGNMENT CORE
        // This calculates the angle to the goal and moves the turret motor
        shooter.setTargetsByDistanceAdjustable(
                currentPose.getX(),
                currentPose.getY(),
                targetX,
                GOAL_Y,
                Math.toDegrees(currentPose.getHeading()),
                true,
                (Poses.getAlliance() == Poses.Alliance.RED) ? 0 : -52, // Your specific offsets
                0, // No manual hood offset
                (Poses.getAlliance() == Poses.Alliance.RED),
                0
        );

        // Update the turret motor PID/Position
        shooter.update(shooter.getCurrentTurretPosition());

        // Optional: Auto-fire if the shooter is ready and the turret is locked
        if (shooter.flywheelVeloReached && timer.getElapsedTimeSeconds() > 2) {
            intake.doTestShooter(); // Feed the balls
        }

        telemetry.addData("Target X", targetX);
        telemetry.addData("Turret Pos", shooter.getCurrentTurretPosition());
        telemetry.addData("Flywheel Ready", shooter.flywheelVeloReached);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopFlywheel();
        intake.resetState();
    }
}