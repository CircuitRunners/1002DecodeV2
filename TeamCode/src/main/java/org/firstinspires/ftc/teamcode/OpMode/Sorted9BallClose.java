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
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    boolean doSort = false;



    // Shot Counting Variables
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    // Field Constants
    private final double RED_GOAL_X = 127.0;
    private final double BLUE_GOAL_X = 17.0;
    private final double GOAL_Y = 127.5;

    private boolean doTransfer = false;
    private boolean goForLaunch = false;

    private PathChain travelToShoot, openGate, intake1, travelBackToShoot1, intake2, travelBackToShoot2, intake3, travelBackToShoot3,park;

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



        // Path 4: Gate back to Shoot
        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide3).getHeading())
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



        park  = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine2).getHeading(), 0.45)
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
                    shooter.setTurretTarget(253, Shooter.TurretMode.FIELD_CENTRIC,follower.getHeading(),0);

                    setPathState();
                }
                break;

            case 1:
                desiredOrder = limelight.detectBallOrder();
                if (desiredOrder != null){
                    doSort = true;
                }
                setPathState();
                break;


            case 2: // Shoot 3 Preloads
                if (!follower.isBusy() ) {
                    shooter.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC,follower.getHeading(),0);
                    handleAutoShooting(currentPose, targetX, 7,0);

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
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot1, true);
                    setPathState();
                }
                break;

            case 5: // Shoot 3 Balls (Cycle 1)
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1) {
                    handleAutoShooting(currentPose, targetX, 4.5,0);
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
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot2, true);
                    setPathState();
                }
                break;

            case 8: // Shoot 3 Balls (Cycle 2)
                if (!follower.isBusy()) {
                    handleAutoShooting(currentPose, targetX, 4.5,0);
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
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot3, true);
                    setPathState();
                }
                break;

            case 11: // Final 3 Balls
                if (!follower.isBusy()) {
                    handleAutoShooting(currentPose, targetX, 4.5,0);
                }
                break;

            case 12: // Final 3 Balls
                if (!follower.isBusy()) {
                    follower.followPath(park,true);
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
    private void handleAutoShooting(Pose pose, double targetX, double timeout, double mannualHoodOffset) {
        // Updated shooting command as requested
        double headingDeg = Math.toDegrees(pose.getHeading());


        if (Poses.getAlliance() == Poses.Alliance.RED) {
            shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), targetX, GOAL_Y, headingDeg, false,0, mannualHoodOffset,true,0);
        }
        else {
            shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), targetX, GOAL_Y, headingDeg, false,0, mannualHoodOffset,false,0);
        }
        //shooter.flywheelVeloReached = false;
        if (doSort){
            intake.prepareAndStartSort();
            if (shooter.flywheelVeloReached){
                intake.setCanShoot(true);
            }
        }

        else {
            trackShotCount(shooter.isBeamBroken());


            // Once RPM and Hood Angle are locked, engage the transfer
            if (shooter.flywheelVeloReached && pathTimer.getElapsedTimeSeconds() >= 3) {
                doTransfer = true;

//            boolean currentBeamState = shooter.isBeamBroken();
//
//            // Increment count on "Falling Edge" (Ball has fully cleared the shooter)
//            if (lastBeamState && !currentBeamState) {
//                ballsShotInState++;
//            }
//            lastBeamState = currentBeamState;
            }


            if (doTransfer && shooter.turretReached) {
                intake.doTransfer();
            }


        }
        // Advance to next state if 3 balls fired OR the safety timer expires
        if (pathTimer.getElapsedTimeSeconds() > timeout || ballsShotInState >= 3) {
            doTransfer = false;
            shooter.stopFlywheel();
            ballsShotInState = 0;
            intake.doIntakeHalt();
            goForLaunch = false;
            intake.setCanShoot(false);
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
        loopTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        limelight = new LimelightCamera(hardwareMap);
        intake.setCanShoot(false);
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

        telemetry.addData("Hub Status", sensors.isHubDisconnected() ? "DISCONNECTED (Error)" :

                (sensors.isHubReady() ? "Ready (Awaiting Start)" : "Waiting for Config..."));




        telemetry.addLine("--- Alliance Selector ---");
        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
        telemetry.addLine("");
        telemetry.addData("Alliance Set", Poses.getAlliance());
        telemetry.addData("Start Pose", Poses.get(Poses.startPoseGoalSide));

        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
        sensors.update();
        lastBeamState = shooter.isBeamBroken();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Must update all hardware/sensors every loop
        loopTimer.resetTimer();
        follower.update();
        pinpoint.update();
        sensors.update();
        shooter.update(shooter.getCurrentTurretPosition());
        intake.update(shooter.isBeamBroken(), desiredOrder,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Beam Status", shooter.isBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Shooter Velo", shooter.getFlywheelVelo());
        telemetry.addData("is up to sped",shooter.flywheelVeloReached);
        telemetry.addData("Balls shot in state:",ballsShotInState);
        telemetry.addData("Loop Time",loopTimer.getElapsedTime());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopFlywheel();
        intake.resetState();
        Poses.savePose(follower.getPose());
    }

    private void trackShotCount(boolean currentBeamState) {
        if (lastBeamState && !currentBeamState) {
            ballsShotInState++;
        }
        lastBeamState = currentBeamState;
    }


}