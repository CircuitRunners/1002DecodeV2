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
@Autonomous(name = "GS 15 Ball ", group = "A", preselectTeleOp = "v2Teleop")
public class FifteenBallClose extends OpMode {

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Timer pathTimer;
    private Timer loopTimer;
    private Shooter shooter;
    private Intake intake;
    private Sensors sensors;

    private int pathState;
    private Poses.Alliance lastKnownAlliance = null;

    boolean veloReached = false;

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
    boolean flywheelLocked = false;

    private PathChain travelToShoot, openGate, intake1, travelBackToShoot1, intake2, travelBackToShoot2, intake3, travelBackToShoot3,intake4,travelBackToShoot4;

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


        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine1).getHeading(), 0.25)
                .build();

        // Path 5: Shoot to Intake 2
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine2).getHeading(), 0.45)
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine2),Poses.get(Poses.fifteenBallOpenGateControlPoint), Poses.get(Poses.openGate)))

                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Math.toRadians(90), 0.55)
                .build();


        // Path 6: Intake 2 back to Shoot
        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.openGate), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        // Path 7: Shoot to Intake 3
        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line3ControlPoint), Poses.get(Poses.pickupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine3).getHeading(), 0.45)
                .build();

        // Path 8: Intake 3 back to final Shoot
        travelBackToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine3),Poses.get(Poses.line3ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.fifteenBallLine4ControlPoint1), Poses.get(Poses.fifteenBallLine4ControlPoint2),Poses.get(Poses.fifteenBallPickupLine4)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.fifteenBallPickupLine4).getHeading(), 0.5)
                .build();

        travelBackToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.fifteenBallPickupLine4),Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.fifteenBallPickupLine4).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading(),0.75,0.35)
                .build();




    }

    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

        boolean isShootingState =
                pathState == 1 ||
                        pathState == 4 ||
                        pathState == 8 ||
                        pathState == 11 ||
                        pathState == 14;

        if (!isShootingState) {
            doTransfer = false;
            goForLaunch = false;
            flywheelLocked = false;
        }

        switch (pathState) {
            case 0: // Travel to Initial Shoot
                //intake.retainBalls();
                if (!follower.isBusy()) {
                    follower.followPath(travelToShoot, true);

                    setPathState();
                }
                break;

            case 1: // Shoot 3 Preloads
                handleAutoShooting(currentPose, targetX, 4.5,0);
                if (!goForLaunch
                        && follower.atParametricEnd()
                        && follower.getVelocity().getMagnitude() < 1) {
                    goForLaunch = true;
                }
                break;

            case 2: // Drive to Intake 1
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState();
                }

                break;


            case 3: // Return to Shoot 1

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5  || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1 )) {
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 4: // Final 3 Balls

                // Stop intake once we're ~45% through the path
                stopIntakeOnceAtT(0.45);

                // Shooter logic owns intake AFTER the stop
                if (intakeStoppedForShooting) {
                    handleAutoShooting(currentPose, targetX, 5.3, 0);
                }

                // Allow feeding once fully settled
                if (intakeStoppedForShooting
                        && !goForLaunch
                        && follower.atParametricEnd()
                        && follower.getVelocity().getMagnitude() < 1) {
                    goForLaunch = true;
                }

                break;

            case 5: // Drive to Intake 2
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake2, false);
                    setPathState();
                }
                break;

            case 6:
                if (!follower.isBusy() || (follower.getVelocity().getMagnitude() < 1 && pathTimer.getElapsedTimeSeconds() > 2)) {
                    follower.followPath(openGate, false);
                    setPathState();
                }
                break;

            case 7: // Return to Shoot 2

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.5  || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1 )) {
                    follower.followPath(travelBackToShoot2, true);
                    setPathState();
                }
                break;

            case 8: // Final 3 Balls

                // Stop intake once we're ~45% through the path
                stopIntakeOnceAtT(0.45);

                // Shooter logic owns intake AFTER the stop
                if (intakeStoppedForShooting) {
                    handleAutoShooting(currentPose, targetX, 5.3, 0);
                }

                // Allow feeding once fully settled
                if (intakeStoppedForShooting
                        && !goForLaunch
                        && follower.atParametricEnd()
                        && follower.getVelocity().getMagnitude() < 1) {
                    goForLaunch = true;
                }

                break;

            case 9: // Drive to Intake 3
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake3, false);
                    setPathState();
                }
                break;

            case 10: // Return to Shoot 3

                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {

                    follower.followPath(travelBackToShoot3, true);
                    setPathState();
                }
                break;


            case 11: // Final 3 Balls

                // Stop intake once we're ~45% through the path
                stopIntakeOnceAtT(0.45);

                // Shooter logic owns intake AFTER the stop
                if (intakeStoppedForShooting) {
                    handleAutoShooting(currentPose, targetX, 5.3, 0);
                }

                // Allow feeding once fully settled
                if (intakeStoppedForShooting
                        && !goForLaunch
                        && follower.atParametricEnd()
                        && follower.getVelocity().getMagnitude() < 1) {
                    goForLaunch = true;
                }

                break;

            case 12: // Drive to Intake 3
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake4, false);
                    setPathState();
                }
                break;

            case 13: // Return to Shoot 3

                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {
                    ;
                    follower.followPath(travelBackToShoot4, true);
                    setPathState();
                }
                break;


            case 14: // Final 3 Balls

                // Stop intake once we're ~45% through the path
                stopIntakeOnceAtT(0.45);

                // Shooter logic owns intake AFTER the stop
                if (intakeStoppedForShooting) {
                    handleAutoShooting(currentPose, targetX, 25, 0);
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

    /**
     * Logic for calculating ballistics, locking turret,
     * and counting exactly 3 shots based on beam break transitions.
     */
//    private void handleAutoShooting(Pose pose, double targetX, double timeout, double mannualHoodOffset) {
//        // Updated shooting command as requested
//        double headingDeg = Math.toDegrees(pose.getHeading());
//
//
//        if (Poses.getAlliance() == Poses.Alliance.RED) {
//            shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), targetX, GOAL_Y, headingDeg, false,0, mannualHoodOffset,true,0);
//        }
//        else {
//            shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), targetX, GOAL_Y, headingDeg, false,0, mannualHoodOffset,false,0);
//        }
//        //shooter.flywheelVeloReached = false;
//
//
//
//
//        // Once RPM and Hood Angle are locked, engage the transfer
//        if (veloReached  && goForLaunch) {
//            doTransfer = true;
//
//
////            boolean currentBeamState = shooter.isBeamBroken();
////
////            // Increment count on "Falling Edge" (Ball has fully cleared the shooter)
////            if (lastBeamState && !currentBeamState) {
////                ballsShotInState++;
////            }
////            lastBeamState = currentBeamState;
//        }
//
//
//        if (doTransfer){
//            trackShotCount(shooter.isBeamBroken());
//            intake.doTestShooter();
//        }
//
//
//        // Advance to next state if 3 balls fired OR the safety timer expires
//        if (pathTimer.getElapsedTimeSeconds() > timeout || ballsShotInState >= 3) {
//            doTransfer = false;
//            shooter.stopFlywheel();
//            ballsShotInState = 0;
//            intake.doIntakeHalt();
//            goForLaunch = false;
//            setPathState();
//            return;
//        }
//    }

    private void handleAutoShooting(
            Pose pose,
            double targetX,
            double timeout,
            double mannualHoodOffset
    ) {
        double headingDeg = Math.toDegrees(pose.getHeading());

        // Always command shooter targets
        if (Poses.getAlliance() == Poses.Alliance.RED) {
            shooter.setTargetsByDistanceAdjustable(
                    pose.getX(), pose.getY(),
                    targetX, GOAL_Y,
                    headingDeg,
                    false, 0,
                    mannualHoodOffset,
                    true, 0
            );
        } else {
            shooter.setTargetsByDistanceAdjustable(
                    pose.getX(), pose.getY(),
                    targetX, GOAL_Y,
                    headingDeg,
                    false, 0,
                    mannualHoodOffset,
                    false, 0
            );
        }

        //  Latch flywheel once it EVER reaches speed
        if (veloReached) {
            flywheelLocked = true;
        }

        //  Only allow feeding when fully ready
        if (flywheelLocked && goForLaunch) {
            doTransfer = true;
        }

        //  Feeding + shot counting
        if (doTransfer) {
            trackShotCount(shooter.isBeamBroken());
            intake.doTestShooter();
        }

        // ⏱ FAILSAFE EXIT (prevents sitting forever)
        if (ballsShotInState >= 3 || pathTimer.getElapsedTimeSeconds() > timeout) {
            resetShootingState();
            shooter.stopFlywheel();
            intake.doIntakeHalt();
            setPathState();
        }
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

    @Override
    public void init() {
        // Bulk reading for loop speed
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        pathTimer = new Timer();
        loopTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry,true);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");
        goForLaunch = false;
    }

    @Override
    public void init_loop() {
        Poses.updateAlliance(gamepad1, telemetry);


        if (Poses.getAlliance() != lastKnownAlliance) {
            follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));
            buildPaths();

            lastKnownAlliance = Poses.getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
            telemetry.addLine("");
        }

//        telemetry.addData("Hub Status", sensors.isHubDisconnected() ? "DISCONNECTED (Error)" :
//
//                (sensors.isHubReady() ? "Ready (Awaiting Start)" : "Waiting for Config..."));




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
        sensors.run();
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Must update all hardware/sensors every loop
        loopTimer.resetTimer();
        follower.update();
        pinpoint.update();

        shooter.update(shooter.getCurrentTurretPosition());
        intake.update(shooter.isBeamBroken(), LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        autonomousPathUpdate();

        veloReached = (Math.abs(shooter.getFlywheelVelo()) > (Math.abs(shooter.getTargetFLywheelVelo()) - (40)) && Math.abs(shooter.getFlywheelVelo()) < (Math.abs(shooter.getTargetFLywheelVelo()) + (40)) && Math.abs(shooter.getTargetFLywheelVelo()) >=1);

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Beam Status", shooter.isBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Shooter Velo", shooter.getFlywheelVelo());
        telemetry.addData("is up to sped",veloReached);
        telemetry.addData("Balls shot in state:",ballsShotInState);
        telemetry.addData("Loop Time",loopTimer.getElapsedTime());
        telemetry.addData("FLywheel Velo",shooter.getFlywheelVelo());
        telemetry.addData("target velo",shooter.getTargetFLywheelVelo());
        telemetry.addData("Go for launch?",goForLaunch);
        telemetry.addData("Path t", follower.getCurrentTValue());
        telemetry.addData("IntakeStopped", intakeStoppedForShooting);
        telemetry.update();
    }

    @Override
    public void stop() {
        sensors.stop();
        shooter.stopFlywheel();
        intake.resetState();
        Poses.savePose(follower.getPose());
    }

//    private void trackShotCount(boolean currentBeamState) {
//        if (lastBeamState && !currentBeamState) {
//            ballsShotInState++;
//        }
//        lastBeamState = currentBeamState;
//    }


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

    private void stopIntakeOnceAtT(double t) {
        if (!intakeStoppedForShooting && follower.getCurrentTValue() >= t && follower.isBusy()) {
            intake.doIntakeHalt();          // ONE-TIME call
            intakeStoppedForShooting = true;
        }
    }


    private void resetShootingState() {
        ballsShotInState = 0;
        doTransfer = false;
        goForLaunch = false;
        beamWasCleared = true;
        lastBeamState = shooter.isBeamBroken();
        pathTimer.resetTimer();
    }


}