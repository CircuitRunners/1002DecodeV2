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

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@Autonomous(name = "GS 9 Ball SORT ", group = "A", preselectTeleOp = "v2Teleop")
public class Sorted9BallClose extends OpMode {

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Timer pathTimer;
    private Timer loopTimer;
    private Shooter shooter;
    private Intake intake;
    private Sensors sensors;
    private LimelightCamera limelight;

    private int pathState;
    private Poses.Alliance lastKnownAlliance = null;
    private LimelightCamera.BallOrder desiredOrder = null;

    boolean veloReached = false;
    boolean flywheelLocked = false;
    double flywheelMannualOffset = 700;

    // Shot Counting Variables
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;
    private boolean beamWasCleared = true;

    // Field Constants
    private final double RED_GOAL_X = 127.0;
    private final double BLUE_GOAL_X = 17.0;
    private final double GOAL_Y = 127.5;

    private boolean doTransfer = false;
    private boolean goForLaunch = false;

    private PathChain travelToShoot, getBallOrder, intake1, travelBackToShoot1, intake2, travelBackToShoot2;

    public void buildPaths() {
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.getMotif).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        getBallOrder = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseGoalSide).getHeading(), Poses.get(Poses.getMotif).getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.controlPointLine1ForShootPose2), Poses.get(Poses.pickupLine1)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine1).getHeading(), 0.25)
                .build();

        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide3).getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine2).getHeading(), 0.45)
                .build();

        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

        // Reset flags if not in a shooting state (Logic from 12-ball)
        boolean isShootingState = (pathState == 2 || pathState == 5 || pathState == 8);
        if (!isShootingState) {
            doTransfer = false;
            goForLaunch = false;
            flywheelLocked = false;
        }

        switch (pathState) {
            case 0: // Travel to Initial Shoot
                if (!follower.isBusy()) {
                    follower.followPath(getBallOrder, true);
                    setPathState();
                }
                break;

            case 1: // Limelight Detection (Sorted logic preserved)
                desiredOrder = limelight.detectBallOrder();
                if (desiredOrder != null && !follower.isBusy()) {
                    follower.followPath(travelToShoot, true);
                    setPathState();
                }
                // Optional: add a timeout here if Limelight doesn't see anything
                break;

            case 2: // Shoot 3 Preloads
                intake.prepareAndStartSort();
                handleAutoShooting(currentPose, targetX, 7.0, 0);
                if (!goForLaunch && follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1 && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    goForLaunch = true;
                }
                break;

            case 3: // Drive to Intake 1
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState();
                }
                break;

            case 4: // Return to Shoot 1
                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {
                    intake.prepareAndStartSort();
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 5: // Shoot 3 Balls (Cycle 1)
                handleAutoShooting(currentPose, targetX, 4.5, 0);
                if (!goForLaunch && follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1) {
                    goForLaunch = true;
                }
                break;

            case 6: // Drive to Intake 2
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake2, false);
                    setPathState();
                }
                break;

            case 7: // Return to Shoot 2
                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {
                    intake.prepareAndStartSort();
                    follower.followPath(travelBackToShoot2, true);
                    setPathState();
                }
                break;

            case 8: // Shoot 3 Balls (Cycle 2)
                handleAutoShooting(currentPose, targetX, 4.5, 0);
                if (!goForLaunch && follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1) {
                    goForLaunch = true;
                }
                break;

            default:
                shooter.stopFlywheel();
                intake.resetState();
                if (!follower.isBusy()) requestOpModeStop();
                break;
        }
    }

    private void handleAutoShooting(Pose pose, double targetX, double timeout, double mannualHoodOffset) {
        double headingDeg = Math.toDegrees(pose.getHeading());

        // Ballistics targeting
        if (Poses.getAlliance() == Poses.Alliance.RED) {
            shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), targetX, GOAL_Y, headingDeg, false, flywheelMannualOffset, mannualHoodOffset, true, 0);
        } else {
            shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), targetX, GOAL_Y, headingDeg, false, flywheelMannualOffset, mannualHoodOffset, false, 0);
        }

        // Latch flywheel (12-ball logic)
        if (veloReached) {
            flywheelLocked = true;
        }

        // Only allow feeding when ready (added intake state check from 9-ball)
        if (flywheelLocked && goForLaunch && (intake.getCurrentIntakeState() == Intake.IntakeState.READY_TO_FIRE)) {
            doTransfer = true;
        }

        if (doTransfer) {
            trackShotCount(shooter.isBeamBroken());
            intake.doTestShooter();
        }

        // Fail-safe exit
        if (ballsShotInState >= 3 || pathTimer.getElapsedTimeSeconds() > timeout) {
            resetShootingState();
            shooter.stopFlywheel();
            intake.doIntakeHalt();
            setPathState();
        }
    }

    private void trackShotCount(boolean currentBeamState) {
        if (!currentBeamState) {
            beamWasCleared = true;
        }
        if (currentBeamState && beamWasCleared) {
            ballsShotInState++;
            beamWasCleared = false;
        }
    }

    private void resetShootingState() {
        ballsShotInState = 0;
        doTransfer = false;
        goForLaunch = false;
        beamWasCleared = true;
        flywheelLocked = false;
        lastBeamState = shooter.isBeamBroken();
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        pathTimer = new Timer();
        loopTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");
        limelight = new LimelightCamera(hardwareMap);

        intake.setCanShoot(false);
        goForLaunch = false;
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
        lastBeamState = shooter.isBeamBroken();
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        loopTimer.resetTimer();
        follower.update();
        pinpoint.update();
        sensors.update();
        shooter.update(shooter.getCurrentTurretPosition());

        // Passing detected colors to intake for sorting
        intake.update(shooter.isBeamBroken(), desiredOrder != null ? desiredOrder : LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        autonomousPathUpdate();

        // Updated veloReached calculation from 12-ball
        veloReached = (Math.abs(shooter.getFlywheelVelo()) > (Math.abs(shooter.getTargetFLywheelVelo()) - 40)
                && Math.abs(shooter.getFlywheelVelo()) < (Math.abs(shooter.getTargetFLywheelVelo()) + 40)
                && Math.abs(shooter.getTargetFLywheelVelo()) >= 1);

        telemetry.addData("Ball Order", desiredOrder);
        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Velo Reached", veloReached);
        telemetry.addData("Intake State", intake.getCurrentIntakeState());
        intake.doSortingTelemetry(sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()),desiredOrder, shooter.isBeamBroken());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopFlywheel();
        intake.resetState();
        Poses.savePose(follower.getPose());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }
}