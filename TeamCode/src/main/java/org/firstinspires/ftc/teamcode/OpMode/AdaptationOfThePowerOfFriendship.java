package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@Autonomous(name = "GS 12 Ball - Fixed Turret", group = "A", preselectTeleOp = "v2Teleop")
public class AdaptationOfThePowerOfFriendship extends OpMode {

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Timer pathTimer;
    private Shooter shooter;
    private Intake intake;
    private Sensors sensors;

    private int pathState;
    private Poses.Alliance lastKnownAlliance = null;

    // Shot Counting Variables
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    // Field Constants
    private final double RED_GOAL_X = 132.0;
    private final double BLUE_GOAL_X = 12.0;
    private final double GOAL_Y = 137.0;

    private PathChain travelToShoot, openGate, intake1, travelBackToShoot1, intake2, travelBackToShoot2, intake3, travelBackToShoot3;

    public void buildPaths() {
        // Path 1: Start to Shoot Position
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseGoalSide).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        // Path 2: Shoot to Intake 1
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.controlPointLine1ForShootPose2), Poses.get(Poses.pickupLine1)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine1).getHeading(), 0.25)
                .build();

        // Path 3: Intake 1 to Gate
        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine1), Poses.get(Poses.pickupLine1ToGateControlPoint), Poses.get(Poses.openGate)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Math.toRadians(90), 0.45)
                .build();

        // Path 4: Gate back to Shoot
        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.openGate), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Poses.get(Poses.shootPositionGoalSide3).getHeading())
                .build();

        // Path 5: Shoot to Intake 2
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine2).getHeading(), 0.45)
                .build();

        // Path 6: Intake 2 back to Shoot
        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        // Path 7: Shoot to Intake 3
        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.Line3ControlPoint), Poses.get(Poses.pickupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine3).getHeading(), 0.45)
                .build();

        // Path 8: Intake 3 back to final Shoot
        travelBackToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine3), Poses.get(Poses.Line3ControlPoint), Poses.get(Poses.shootPositionGoalSide2LasthHot)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.shootPositionGoalSide2LasthHot).getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

        switch (pathState) {
            case 0: // Travel to Initial Shoot
                intake.retainBalls();
                if (!follower.isBusy()) {
                    follower.followPath(travelToShoot, true);
                    setPathState();
                }
                break;

            case 1: // Shoot 3 Preloads
                handleAutoShooting(currentPose, targetX, 3.5);
                break;

            case 2: // Drive to Intake 1
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState();
                }
                break;

            case 3: // Gate logic
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(openGate, true);
                    setPathState();
                }
                break;

            case 4: // Return to Shoot 1
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 5: // Shoot 3 Balls (Cycle 1)
                handleAutoShooting(currentPose, targetX, 2.5);
                break;

            case 6: // Drive to Intake 2
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(intake2, false);
                    setPathState();
                }
                break;

            case 7: // Return to Shoot 2
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot2, true);
                    setPathState();
                }
                break;

            case 8: // Shoot 3 Balls (Cycle 2)
                handleAutoShooting(currentPose, targetX, 2.5);
                break;

            case 9: // Drive to Intake 3
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(intake3, false);
                    setPathState();
                }
                break;

            case 10: // Return to Shoot 3
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot3, true);
                    setPathState();
                }
                break;

            case 11: // Final 3 Balls
                handleAutoShooting(currentPose, targetX, 2.5);
                break;

            default:
                shooter.stopFlywheel();
                intake.resetIndexer();
                if (!follower.isBusy()) requestOpModeStop();
                break;
        }
    }

    /**
     * Logic for calculating ballistics, locking turret,
     * and counting exactly 3 shots based on beam break transitions.
     */
    private void handleAutoShooting(Pose pose, double targetX, double timeout) {
        // Maintain Fixed Turret at 0 (Facing forward)
        shooter.setShooterTarget(
                pose.getX(), pose.getY(), targetX, GOAL_Y,
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                 Math.toDegrees(pose.getHeading()),
                true
        );

        // Once RPM and Hood Angle are locked, engage the transfer
        if (shooter.flywheelVeloReached && shooter.hoodReached) {
            intake.intake();

            boolean currentBeamState = sensors.isBeamBroken();

            // Increment count on "Falling Edge" (Ball has fully cleared the shooter)
            if (lastBeamState && !currentBeamState) {
                ballsShotInState++;
            }
            lastBeamState = currentBeamState;
        }

        // Advance to next state if 3 balls fired OR the safety timer expires
        if (ballsShotInState >= 3 || pathTimer.getElapsedTimeSeconds() > timeout) {
            ballsShotInState = 0;
            intake.resetIndexer();
            setPathState();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        // Bulk reading for loop speed
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");
    }

    @Override
    public void init_loop() {
        Poses.updateAlliance(gamepad1, telemetry);
        if (Poses.getAlliance() != lastKnownAlliance) {
            follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));
            buildPaths();
            lastKnownAlliance = Poses.getAlliance();
        }
        sensors.update();
        lastBeamState = sensors.isBeamBroken();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Must update all hardware/sensors every loop
        follower.update();
        pinpoint.update();
        sensors.update();
        shooter.update(sensors.getFlywheelVelo(), sensors.getSketchTurretPosition());

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Beam Status", sensors.isBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Shooter Velo", sensors.getFlywheelVelo());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopFlywheel();
        intake.resetIndexer();
        Poses.savePose(follower.getPose());
    }
}