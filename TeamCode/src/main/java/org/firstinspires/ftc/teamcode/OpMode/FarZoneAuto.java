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
@Autonomous(name = "FarZoneAuto", group = "A", preselectTeleOp = "v2Teleop")
public class FarZoneAuto extends OpMode {

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Timer pathTimer;
    private Timer loopTimer; // Added loop timer for consistency
    private Shooter shooter;
    private Intake intake;
    private Sensors sensors;

    private int pathState;
    private Poses.Alliance lastKnownAlliance = null;

    // Logic Flags from 12-ball
    boolean veloReached = false;
    boolean flywheelLocked = false;
    boolean goForLaunch = false;
    private boolean doTransfer = false;

    // Shot Counting Variables
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;
    private boolean beamWasCleared = true;

    // One-shot latch: stops intake exactly once before shooting
    private boolean intakeStoppedForShooting = false;

    // Field Constants
    private final double RED_GOAL_X = 126;
    private final double BLUE_GOAL_X = 11;
    private final double GOAL_Y = 132;


    private PathChain travelToShoot, humanPlayerIntake, travelBackToShoot1, intakeLine, travelBackToShoot2;

    public void buildPaths() {
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseFarSide), Poses.get(Poses.shootPositionFarSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseFarSide).getHeading(), Poses.get(Poses.shootPositionFarSide).getHeading())
                .build();

        humanPlayerIntake = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseFarSide), Poses.get(Poses.humanPlayerIntake)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseFarSide).getHeading(), Poses.get(Poses.humanPlayerIntake).getHeading(), 0.25)
                .addPath(new BezierLine(Poses.get(Poses.humanPlayerIntake), Poses.get(Poses.backUpPoint)))
                .setLinearHeadingInterpolation(Poses.get(Poses.humanPlayerIntake).getHeading(), Poses.get(Poses.backUpPoint).getHeading(), 0.25)
                .addPath(new BezierLine(Poses.get(Poses.backUpPoint), Poses.get(Poses.humanPlayerIntakeRam)))
                .setLinearHeadingInterpolation(Poses.get(Poses.backUpPoint).getHeading(), Poses.get(Poses.humanPlayerIntakeRam).getHeading(), 0.25)
                .build();

        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.humanPlayerIntake), Poses.get(Poses.startPoseFarSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.humanPlayerIntake).getHeading(), Poses.get(Poses.startPoseFarSide).getHeading())
                .build();

        intakeLine = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.startPoseFarSide), Poses.get(Poses.intake3ControlPoint), Poses.get(Poses.pickupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseFarSide).getHeading(), Poses.get(Poses.pickupLine3).getHeading(), 0.45)
                .build();

        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine3), Poses.get(Poses.startPoseFarSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.startPoseFarSide).getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

        // Auto-kill transfer if not in a shooting state
        boolean isShootingState = (pathState == 1 || pathState == 6 || pathState == 9 || pathState == 14);
        if (!isShootingState) {
            doTransfer = false;
            goForLaunch = false;
            flywheelLocked = false;
        }

        switch (pathState) {
            case 0: // Travel to Initial Shoot
                shooter.setTurretTarget(targetX == RED_GOAL_X ? 310 : -70, Shooter.TurretMode.ROBOT_CENTRIC,0,0);
                if (!follower.isBusy()) {
                    follower.followPath(travelToShoot, false);
                    setPathState();
                }
                setPathState();
                break;

            case 1: // Shoot 3 Preloads
                handleAutoShooting(currentPose, targetX, 6.5, 0);
                if (!goForLaunch
                        && (follower.getVelocity().getMagnitude() < 1.8) && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    goForLaunch = true;
                }
                break;

            case 2: // Drive to Intake
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(humanPlayerIntake, true);
                    setPathState(5); // Skipping to 5 based on your original logic
                }
                break;

            case 5: // Return to Shoot 1
                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 6: // Shoot 3 Balls (Cycle 1)
                handleAutoShooting(currentPose, targetX, 6.5, 0);
                if (!goForLaunch
                        && (follower.getVelocity().getMagnitude() < 1.8) && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    goForLaunch = true;
                }
                break;

            case 7: // Drive to Intake 2
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intakeLine, false);
                    setPathState();
                }
                break;

            case 8: // Return to Shoot 2
                if (!follower.isBusy() || (follower.atParametricEnd() && follower.getVelocity().getMagnitude() < 1)) {
                    follower.followPath(travelBackToShoot2, true);
                    setPathState();
                }
                break;

            case 9: // Shoot 3 Balls (Cycle 2)
                handleAutoShooting(currentPose, targetX, 6.5, 0);
                if (!goForLaunch
                        && (follower.getVelocity().getMagnitude() < 1.8) && pathTimer.getElapsedTimeSeconds() > 0.5) {
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
                    false, -58,
                    mannualHoodOffset ,
                    false, 0
            );
        }

        //  Latch flywheel once it EVER reaches speed



        //  Only allow feeding when fully ready
        if (veloReached && goForLaunch) {
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
            intakeStoppedForShooting = false;
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
        flywheelLocked = false;
        beamWasCleared = true;
        lastBeamState = shooter.isBeamBroken();
        pathTimer.resetTimer();
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
            follower.setStartingPose(Poses.get(Poses.startPoseFarSide));
            buildPaths();

            lastKnownAlliance = Poses.getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
            telemetry.addLine("");
        }

    //    telemetry.addData("Hub Status", sensors.isHubDisconnected() ? "DISCONNECTED (Error)" :

      //          (sensors.isHubReady() ? "Ready (Awaiting Start)" : "Waiting for Config..."));




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
        loopTimer.resetTimer();
        follower.update();
        pinpoint.update();

        shooter.update(shooter.getCurrentTurretPosition());

        // Update sorting/intake (using default order as FarSide usually doesn't sort)
        intake.update(shooter.isBeamBroken(), LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE,
                sensors.getDetectedColor(sensors.colorSensor1),
                sensors.getDetectedColor(sensors.colorSensor2),
                sensors.getDetectedColor(sensors.colorSensor3)
        );

        autonomousPathUpdate();

        // Target Velo logic from 12-ball
        veloReached = (Math.abs(shooter.getFlywheelVelo()) > (Math.abs(shooter.getTargetFLywheelVelo()) - 40) &&
                Math.abs(shooter.getFlywheelVelo()) < (Math.abs(shooter.getTargetFLywheelVelo()) + 40) &&
                Math.abs(shooter.getTargetFLywheelVelo()) >= 1);

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Shooter Velo", shooter.getFlywheelVelo());
        telemetry.addData("Velo Reached", veloReached);
        telemetry.addData("Go for launch?", goForLaunch);
        telemetry.addData("Path t", follower.getCurrentTValue());
        telemetry.addData("IntakeStopped", intakeStoppedForShooting);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopFlywheel();
        intake.resetState();
        Poses.savePose(follower.getPose());
    }

    private void stopIntakeOnceAtT(double t) {
        if (!intakeStoppedForShooting && follower.getCurrentTValue() >= t && follower.isBusy()) {
            intake.doIntakeHalt();          // ONE-TIME call
            intakeStoppedForShooting = true;
        }
    }
}