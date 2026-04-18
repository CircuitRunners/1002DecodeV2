package org.firstinspires.ftc.teamcode.OpMode.Auto;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
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
    private NewShooter shooter;
    private Intake intake;
    private Sensors sensors;
    private LimelightCamera limelight;

    private List<LynxModule> allHubs;


    private int pathState;
    private Poses.Alliance lastKnownAlliance = null;
    private LimelightCamera.BallOrder desiredOrder = null;

    boolean veloReached = false;
    boolean flywheelLocked = false;
    double flywheelMannualOffset = 700;

    // Shot Counting Variables
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;


    // Field Constants
    private final double RED_GOAL_X = 126;
    private final double BLUE_GOAL_X = 15;
    private final double GOAL_Y = 132;


    private boolean doTransfer = false;
    private boolean goForLaunch = false;

    // One-shot latch: stops intake exactly once before shooting
    private boolean intakeStoppedForShooting = false;

    private boolean sortStarted = false;
    private PathChain travelToShoot, getBallOrder, intake1, travelBackToShoot1, intake2, travelBackToShoot2;

    public void buildPaths() {

        getBallOrder = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPose15Ball), Poses.get(Poses.getMotif)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPose15Ball).getHeading(), Poses.get(Poses.getMotif).getHeading())
                .build();

        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.getMotif), Poses.get(Poses.shootPositionGoalSide15BallTangent)))
                .setLinearHeadingInterpolation(Poses.get(Poses.getMotif).getHeading(), Poses.get(Poses.shootPositionGoalSide15BallTangent).getHeading())
                .build();



        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide15BallTangent), Poses.get(Poses.pickupLineOne15Ball)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide15BallTangent).getHeading(), Poses.get(Poses.pickupLineOne15Ball).getHeading(), 0.25)
                .build();

        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide15BallTangent)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide15BallTangent).getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide15BallTangent), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.line2ControlPointTangent), Poses.get(Poses.shootPositionGoalSide15BallTangent)))
                .setTangentHeadingInterpolation().setReversed()
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
                shooter.setTurretTargetPosition(Poses.getAlliance() == Poses.Alliance.RED ? 105 : -75);
                if (!follower.isBusy()) {
                    follower.followPath(getBallOrder, true);
                    setPathState();
                }
                break;

            case 1: // Limelight Detection (Sorted logic preserved)
                desiredOrder = limelight.detectBallOrder();
                if ((desiredOrder != null || pathTimer.getElapsedTimeSeconds() >= 5) && !follower.isBusy()) {
                    follower.followPath(travelToShoot, true);
                    setPathState();
                }
                // Optional: add a timeout here if Limelight doesn't see anything
                break;

            case 2: // Shoot 3 Preloads
                doSort();
                if (intake.getCurrentIntakeState() == Intake.SimpleSortState.READY || pathTimer.getElapsedTimeSeconds() > 9.5) {
                    handleAutoShooting(currentPose, targetX, 4.1, 0, false);
                }
                if (!goForLaunch && follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1 && pathTimer.getElapsedTimeSeconds() > 1.5) {

                    goForLaunch = true;
                }
                break;

            case 3: // Drive to Intake 1
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, false);
                    setPathState();
                }
                break;



            case 4: // Return to Shoot 1
                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {
                    intake.doIntake();
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 5: // Final 3 Balls

                // Stop intake once we're ~45% through the path
                stopIntakeOnceAtT(0.35);

                // Shooter logic owns intake AFTER the stop
                if (intakeStoppedForShooting) {
                    doSort();
                }

                if (intake.getCurrentIntakeState() == Intake.SimpleSortState.READY || pathTimer.getElapsedTimeSeconds() > 7.5) {
                    handleAutoShooting(currentPose, targetX, 3.5, 0, false);
                }


                // Allow feeding once fully settled
                if (intakeStoppedForShooting
                        && !goForLaunch
                        && follower.atParametricEnd()
                        && follower.getVelocity().getMagnitude() < 1) {

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
                   intake.doIntake();
                    follower.followPath(travelBackToShoot2, true);
                    setPathState();
                }
                break;

            case 8: // Shoot 3 Balls (Cycle 2)
                stopIntakeOnceAtT(0.35);

                // Shooter logic owns intake AFTER the stop
                if (intakeStoppedForShooting) {
                    doSort();
                }

                if (intake.getCurrentIntakeState() == Intake.SimpleSortState.READY || pathTimer.getElapsedTimeSeconds() > 7.5) {
                    handleAutoShooting(currentPose, targetX, 25, 0, true);
                }


                // Allow feeding once fully settled
                if (intakeStoppedForShooting
                        && !goForLaunch
                        && follower.atParametricEnd()
                        && follower.getVelocity().getMagnitude() < 1) {

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

    private void handleAutoShooting(
            Pose pose,
            double targetX,
            double timeout,
            double mannualHoodOffset, boolean lastTime
    ) {
        double headingDeg = Math.toDegrees(pose.getHeading());

        // Always command shooter targets
        if (Poses.getAlliance() == Poses.Alliance.RED) {
            shooter.setTargetsByDistanceAdjustable(
                    pose.getX(), pose.getY(),
                    targetX, GOAL_Y,
                    headingDeg,
                    true, -6,
                    mannualHoodOffset,
                    true, 0
            );
        } else {
            shooter.setTargetsByDistanceAdjustable(
                    pose.getX(), pose.getY(),
                    targetX, GOAL_Y,
                    headingDeg,
                    true, -58,
                    mannualHoodOffset,
                    false, 0
            );
        }

        //  Only allow feeding when fully ready
        if (veloReached && goForLaunch) {
            doTransfer = true;
        }

        //  Feeding + shot counting
        if (doTransfer) {
            if (!lastTime) {
                trackShotCount(shooter.isBeamBroken());
            }
            intake.doTestShooter();
        }

        // FAILSAFE EXIT (prevents sitting forever)
        if (ballsShotInState >= 3 || pathTimer.getElapsedTimeSeconds() > timeout) {
            resetShootingState();
            sortStarted = false;
            shooter.stopFlywheel();
            intake.doIntakeHalt();
            intakeStoppedForShooting = false;
            setPathState();
        }
    }

    private boolean beamWasCleared = true; // Track full cycle

    private void trackShotCount(boolean currentBeamState) {
        if (!currentBeamState) { // beam clear
            beamWasCleared = true;
        }

        if (currentBeamState && beamWasCleared) { // beam broken after clearing
            ballsShotInState++;
            beamWasCleared = false; // reset until beam clears again
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        pathTimer = new Timer();
        loopTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        intake = new Intake(hardwareMap, telemetry);
        shooter = new NewShooter(hardwareMap, telemetry,true);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");
        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();
        intake.setCanShoot(false);
        goForLaunch = false;
    }

    @Override
    public void init_loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        Poses.updateAlliance(gamepad1, telemetry);


        if (Poses.getAlliance() != lastKnownAlliance) {
            follower.setStartingPose(Poses.get(Poses.startPose15Ball));
            buildPaths();

            lastKnownAlliance = Poses.getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
            telemetry.addLine("");
        }

//        telemetry.addData("Hub Status", sensors.isHubDisconnected() ? "DISCONNECTED (Error)" :
//
//                (sensors.isHubReady() ? "Ready (Awaiting Start)" : "Waiting for Config..."));


        sensors.update();


        telemetry.addLine("--- Alliance Selector ---");
        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
        telemetry.addLine("");
        telemetry.addData("Alliance Set", Poses.getAlliance());
        telemetry.addData("Start Pose", Poses.get(Poses.startPoseGoalSide));

        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        lastBeamState = shooter.isBeamBroken();
    }

    @Override
    public void start() {
        //sensors.run();
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        loopTimer.resetTimer();
        follower.update();
        pinpoint.update();
        sensors.update();

        shooter.update(shooter.getCurrentTurretPosition());

        // Passing detected colors to intake for sorting
        intake.updateProx(sensors.getColor1Proximity(),
                sensors.getColor2Proximity(),
                sensors.getColor3Proximity());


        autonomousPathUpdate();

        // Updated veloReached calculation from 12-ball
        veloReached = (Math.abs(shooter.getFlywheelVelo()) > (Math.abs(shooter.getTargetFLywheelVelo()) - 40)
                && Math.abs(shooter.getFlywheelVelo()) < (Math.abs(shooter.getTargetFLywheelVelo()) + 40)
                && Math.abs(shooter.getTargetFLywheelVelo()) >= 1);

        telemetry.addData("Ball Order", desiredOrder);
        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Path t", follower.getCurrentTValue());
        telemetry.addData("IntakeStopped", intakeStoppedForShooting);
        telemetry.addData("Velo Reached", veloReached);
        telemetry.addData("Intake State", intake.getCurrentIntakeState());
        intake.doSortingTelemetry(sensors.getDetectedColor(sensors.colorSensor1),
                sensors.getDetectedColor(sensors.colorSensor2),
                sensors.getDetectedColor(sensors.colorSensor3),desiredOrder, shooter.isBeamBroken());
        telemetry.addData("loop time",loopTimer.getElapsedTime());

        telemetry.update();
    }

    @Override
    public void stop() {
        //sensors.stop();
        shooter.stopFlywheel();
        intake.resetState();
        limelight.limelightCamera.pause();
        Poses.savePose(follower.getPose());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        intakeStoppedForShooting = false;
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
        intakeStoppedForShooting = false;
    }

    private void stopIntakeOnceAtT(double t) {
        if (!intakeStoppedForShooting && follower.getCurrentTValue() >= t && follower.isBusy()) {
            intake.doIntakeHalt();          // ONE-TIME call
            intakeStoppedForShooting = true;
        }
    }

    private void doSort(){
        String currentPattern = null;
        if (pathState == 2 || pathState == 5) {
            currentPattern = "PPG";
            if (!sortStarted){
                intake.startSimpleSort("PPG", desiredOrder);
                sortStarted = true;
            }

        } else if (pathState == 8) {
            currentPattern = "GPG";
            if (!sortStarted){
                intake.startSimpleSort("GPG", desiredOrder);
                sortStarted = true;
            }
        }

    }
}