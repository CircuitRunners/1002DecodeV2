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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Configurable
@Autonomous(name = "Test Shoot While Moving", group = "A", preselectTeleOp = "v2Teleop")
public class testShootWhileMovingAuto extends OpMode {

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

    private PathChain travelToShoot, openGate, intake1, travelBackToShoot1, intake2, travelBackToShootFromGate, intake3, travelBackToShootFromIntake1, travelBackToShootFromIntake3, park;

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
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.openGateAutoIntake)))
                .setConstantHeadingInterpolation(Poses.get(Poses.openGateAutoIntake).getHeading())
                .build();

        // Path 6: Intake 2 back to Shoot
        travelBackToShootFromGate = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.openGateAutoIntake), Poses.get(Poses.openGateAutoIntakeControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.openGateAutoIntake).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
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
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 0.05) {
                    shootWhileMoving(4.5);
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
                    shootWhileMoving(4.5);
                    setPathState();
                }
                break;

            case 7:
            case 4: // Shoot 3 Balls (Cycle 1)
                if (!follower.isBusy()) {
                    follower.followPath(openGate, true);
                    setPathState();
                }
                break;

            case 8:
            case 5: // WAIT at Gate (2.5s)
                intake.doIntake(); // keep intaking while stalled

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    setPathState();
                }
                break;

            case 9:
            case 6: // Return to Shoot 2
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShootFromGate, true);
                    shootWhileMoving(4.5);
                    setPathState();
                }
                break;


            case 10:
                intake.doIntake();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState();
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShootFromIntake1, true);
                    shootWhileMoving(4.5);
                    setPathState();
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
    private void shootWhileMoving(double timeout) {
        // Updated shooting command as requested
        //double headingDeg = Math.toDegrees(pose.getHeading());

        if (Poses.getAlliance() == Poses.Alliance.RED) {
            shooter.calculateIterativeShot(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), RED_GOAL_X,GOAL_Y,pinpoint.getVelX(DistanceUnit.INCH),pinpoint.getVelY(DistanceUnit.INCH),pinpoint.getHeading(AngleUnit.DEGREES),true,450,0,0);
        }
        else {
            shooter.calculateIterativeShot(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), RED_GOAL_X,GOAL_Y,pinpoint.getVelX(DistanceUnit.INCH),pinpoint.getVelY(DistanceUnit.INCH),pinpoint.getHeading(AngleUnit.DEGREES),false,450,0,0);
        }
        //shooter.flywheelVeloReached = false;


        // Once RPM and Hood Angle are locked, engage the transfer
        if (shooter.flywheelVeloReached  && pathTimer.getElapsedTimeSeconds() >=3) {
            doTransfer = true;

//            boolean currentBeamState = shooter.isBeamBroken();
//
//            // Increment count on "Falling Edge" (Ball has fully cleared the shooter)
//            if (lastBeamState && !currentBeamState) {
//                ballsShotInState++;
//            }
//            lastBeamState = currentBeamState;
        }

        if (doTransfer){
            intake.doTransfer();
        }


        // Advance to next state if 3 balls fired OR the safety timer expires
        if (pathTimer.getElapsedTimeSeconds() > timeout) {
            doTransfer = false;
            shooter.stopFlywheel();
            ballsShotInState = 0;
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
        follower.update();
        pinpoint.update();
        sensors.update();
        shooter.update(sensors.getFlywheelVelo(), shooter.getCurrentTurretPosition());

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Fired", ballsShotInState);
        telemetry.addData("Beam Status", shooter.isBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Shooter Velo", sensors.getFlywheelVelo());
        telemetry.addData("is up to sped",shooter.flywheelVeloReached);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopFlywheel();
        intake.resetState();
        Poses.savePose(follower.getPose());
    }
}