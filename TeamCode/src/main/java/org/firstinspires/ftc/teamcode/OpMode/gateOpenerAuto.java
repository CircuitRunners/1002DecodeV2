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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Configurable
@Autonomous(name = "GS 12 Ball - JAMES", group = "A", preselectTeleOp = "v2Teleop")
public class gateOpenerAuto extends OpMode {

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
    private final double RED_GOAL_X = 127.0;
    private final double BLUE_GOAL_X = 17.0;
    private final double GOAL_Y = 127.5;

    private boolean doTransfer = false;
    private boolean intakeStoppedForShooting = false;
    private boolean goForLaunch = false;
    boolean veloReached = false;
    boolean flywheelLocked = false;




    private PathChain travelToShoot, openGate, intakeGate,intake1, travelBackToShoot1, intake2, travelBackToShootFromGate, intake3, travelBackToShootFromIntake1, travelBackToShootFromIntake3, park;

    public void buildPaths() {
        // Path 1: Start to Shoot Position
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseGoalSide).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        // Path 2: Shoot to Intake 1
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine1).getHeading(), 0.25)
                .build();

        // Path 3: Travel back to shoot
        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine2), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.openGateHighCycleControlPoint),Poses.get(Poses.openGateHighCycle)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.openGateHighCycle).getHeading())
                .build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.openGateHighCycle), Poses.get(Poses.intakeFromGateHighCycle)))
                .setLinearHeadingInterpolation(Poses.get(Poses.openGateHighCycle).getHeading(), Poses.get(Poses.intakeFromGateHighCycle).getHeading())
                .build();

        // Path 6: Intake 2 back to Shoot
        travelBackToShootFromGate = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.intakeFromGateHighCycle), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.intakeFromGateHighCycle).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.controlPointLine1ForShootPose2), Poses.get(Poses.pickupLine1)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine1).getHeading(), 0.45)
                .build();

        travelBackToShootFromIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine1), Poses.get(Poses.controlPointLine1ForShootPose2), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

        intake3  = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line3ControlPoint), Poses.get(Poses.pickupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine3).getHeading(), 0.45)
                .build();

        travelBackToShootFromIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine3), Poses.get(Poses.line3ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

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
                    follower.followPath(intake2, true);
                    setPathState();
                }
                break;

            case 3: // Gate logic
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 4: // Return to Shoot 1
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 0.05) {
                    handleAutoShooting(currentPose, targetX, 4.5,0);
                }
                break;


            case 5: // Shoot 3 Balls (Cycle 1)
                if (!follower.isBusy()) {
                    follower.followPath(openGate, true);
                    setPathState(67);
                }
                break;

            case 67:
                if (!follower.isBusy()){
                    follower.followPath(intakeGate, true);
                    setPathState(6);
                }
            case 6: // WAIT at Gate (2.5s)
                intake.doIntake(); // keep intaking while stalled

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    setPathState();
                }
                break;


            case 7: // Return to Shoot 2
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShootFromGate, true);
                    setPathState();
                }
                break;


            case 8: // Shoot 3 Balls (Cycle 2)
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

            case 9:
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake3, true);
                    setPathState();
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShootFromIntake3, true);
                    setPathState();
                }
                break;

            case 11: // Return to Shoot 1
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

            case 12:
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState();
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShootFromIntake1, true);
                    setPathState();
                }
                break;

            case 14: // Return to Shoot 1
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
                    false, -52,
                    mannualHoodOffset ,
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
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }
    private void stopIntakeOnceAtT(double t) {
        if (!intakeStoppedForShooting && follower.getCurrentTValue() >= t && follower.isBusy()) {
            intake.doIntakeHalt();          // ONE-TIME call
            intakeStoppedForShooting = true;
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
        lastBeamState = shooter.isBeamBroken();
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
        shooter = new Shooter(hardwareMap, telemetry,true);
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
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
            telemetry.addLine("");
        }

       // telemetry.addData("Hub Status", sensors.isHubDisconnected() ? "DISCONNECTED (Error)" :

        //        (sensors.isHubReady() ? "Ready (Awaiting Start)" : "Waiting for Config..."));




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
        follower.update();
        pinpoint.update();

        shooter.update(shooter.getCurrentTurretPosition());

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Beam Status", shooter.isBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Shooter Velo", shooter.getFlywheelVelo());
        telemetry.addData("is up to sped",shooter.flywheelVeloReached);
        telemetry.update();
    }

    @Override
    public void stop() {
        sensors.stop();
        shooter.stopFlywheel();
        intake.resetState();
        Poses.savePose(follower.getPose());
    }
}