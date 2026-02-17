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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Configurable
@Autonomous(name = "GS 15 - 1 Gate Cycle", group = "A", preselectTeleOp = "v2Teleop")

public class Fifteen3lines extends OpMode {

        private Follower follower;
        private GoBildaPinpointDriver pinpoint;
        private Timer pathTimer;
        private ElapsedTime intakeTimer;
        private Shooter shooter;
        private Intake intake;
        private Sensors sensors;

        private List<LynxModule> allHubs;


        private int pathState;
        private Poses.Alliance lastKnownAlliance = null;

        // Shot Counting Variables
        private int ballsShotInState = 0;
        private boolean lastBeamState = false;

        // Field Constants
        private final double RED_GOAL_X = 127;
        private final double BLUE_GOAL_X = 13;
        private final double GOAL_Y = 132;

        public static double startX = 0;
        public static double startY = 0;

        private boolean doTransfer = false;
        private boolean intakeStoppedForShooting = false;
        private boolean goForLaunch = false;
        boolean veloReached = false;
        // boolean flywheelLocked = false;




        private PathChain travelToShoot, intake2Sussy,openGate,travelBackToShootFromGateLastTime,sigmaCycle,ramGate,intake1, travelBackToShoot2, intake2, travelBackToShootFromGate, intake3, travelBackToShootFromIntake1, travelBackToShootFromIntake3, superMegaTechGateCycle;

        public void buildPaths() {
            // Path 1: Start to Shoot Position
            travelToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.startPose15Ball), Poses.get(Poses.shootPositionGoalSide15Ball)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.startPose15Ball).getHeading(), Poses.get(Poses.pickupLine1).getHeading())
                    .build();



            // Path 2: Shoot to Intake 1
            intake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide15Ball), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.pickupLine2)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            intake2Sussy = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide15Ball), new Pose(40,54,180)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .addPath(new BezierLine (new Pose(40,54,180), Poses.get(Poses.pickupLine2)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            // Path 3: Travel back to shoot
            travelBackToShoot2 = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide15Ball)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide15Ball), Poses.get(Poses.openGateHighCycleControlPoint),Poses.get(Poses.openGateHighCycle)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.openGateHighCycle).getHeading())
                    .build();

            ramGate = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide15Ball), Poses.get(Poses.openGateHighCycleControlPoint),Poses.get(Poses.openGateHighCycle)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.openGateHighCycle).getHeading())
                    .build();

            sigmaCycle  = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide15Ball), Poses.get(Poses.openGateHighCycleControlPoint),Poses.get(Poses.openGateHighCycle)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.openGateHighCycle).getHeading())
                    .addPath(new BezierLine(Poses.get(Poses.openGateHighCycle), Poses.get(Poses.intakeFromGateHighCycle)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.openGateHighCycle).getHeading(), Poses.get(Poses.intakeFromGateHighCycle).getHeading())
//                    .addPath(new BezierLine(Poses.get(Poses.intakeFromGateHighCycle), Poses.get(Poses.openGateRamTech)))
//                    .setLinearHeadingInterpolation(Poses.get(Poses.intakeFromGateHighCycle).getHeading(), Poses.get(Poses.openGateRamTech).getHeading())

                    .build();

            superMegaTechGateCycle = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.openGateHighCycle), Poses.get(Poses.intakeFromGateHighCycle)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.openGateHighCycle).getHeading(), Poses.get(Poses.intakeFromGateHighCycle).getHeading())
                    .addPath(new BezierLine(Poses.get(Poses.intakeFromGateHighCycle), Poses.get(Poses.openGateRamTech)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.intakeFromGateHighCycle).getHeading(), Poses.get(Poses.openGateRamTech).getHeading())
                    .build();

            // Path 6: Intake 2 back to Shoot
            travelBackToShootFromGate = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.intakeFromGateHighCycle), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide15Ball)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.openGateRamTech).getHeading(), Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            travelBackToShootFromGateLastTime = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.openGateRamTech), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide15Ball)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.openGateRamTech).getHeading(), Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide15Ball), Poses.get(Poses.pickupLineOne15Ball)))
                    .setConstantHeadingInterpolation( Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            travelBackToShootFromIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.pickupLineOne15Ball), Poses.get(Poses.shootPositionGoalSide15Ball)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            intake3  = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide15Ball), Poses.get(Poses.line3ControlPoint), Poses.get(Poses.pickupLine3)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            travelBackToShootFromIntake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.get(Poses.pickupLine3), Poses.get(Poses.line3ControlPoint), Poses.get(Poses.shootPositionGoalSide15Ball)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

        }

        public void autonomousPathUpdate() {
            Pose currentPose = follower.getPose();
            double targetX = (Poses.getAlliance() == Poses.Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;

            switch (pathState) {
                case 0: // Travel to Initial Shoot
                    //intake.retainBalls();
                    if (!follower.isBusy()) {
                        shooter.setTurretTarget(targetX == RED_GOAL_X ? 315 : 45, Shooter.TurretMode.ROBOT_CENTRIC,0,0);
                        follower.followPath(travelToShoot, false);
                        setPathState();
                    }
                    break;

                case 1: // Shoot 3 Preloads
                    handleAutoShooting(currentPose, targetX, 4.9,0,false);
                    if (!goForLaunch
                            && (follower.getVelocity().getMagnitude() < 1.8) && pathTimer.getElapsedTimeSeconds() > 0.5) {
                        goForLaunch = true;
                    }
                    break;

                case 2: // Drive to Intake 1
                    intake.doIntake();
                    if (!follower.isBusy()) {
//                        follower.followPath(intake2, false);
                        follower.followPath(intake2, false);
                        setPathState();
                    }
                    break;

                case 3: // Gate logic
                    intake.doIntake();
                    if (!follower.isBusy() || (follower.getVelocity().getMagnitude() <=1.4 && pathTimer.getElapsedTimeSeconds() > 0.7)) {
                        follower.followPath(travelBackToShoot2, false);
                        setPathState();
                    }
                    break;

                case 4: // Return to Shoot 1
                    stopIntakeOnceAtT(0.45);

                    // Shooter logic owns intake AFTER the stop
                    if (intakeStoppedForShooting) {
                        handleAutoShooting(currentPose, targetX, 4, 0,false);
                    }

                    // Allow feeding once fully settled
                    if (intakeStoppedForShooting
                            && !goForLaunch
                            && follower.atParametricEnd()
                            && follower.getVelocity().getMagnitude() < 1.8) {
                        goForLaunch = true;
                    }
                    break;



                case 5: // Shoot 3 Balls (Cycle 1)
                    if (!follower.isBusy()) {
                        intake.doIntake();
//                        follower.followPath(ramGate, false);
//                        setPathState(41);
                        follower.followPath(sigmaCycle,true);
                        setPathState();
                    }

//                    if (!follower.isBusy()) {
//                        intake.doIntake();
//                        follower.followPath(sigmaCycle, false);
//                        setPathState(6);
//                    }
                    break;

                case 41:
                    intake.doIntake();
                    if (!follower.isBusy()){
                        follower.followPath(superMegaTechGateCycle,false);
                        setPathState(6);

                    }
                    break;


                case 6: // WAIT at Gate (2.5s)
                    intake.doIntake(); // keep intaking while stalled

                    if ((pathTimer.getElapsedTimeSeconds() >= 5 && follower.getVelocity().getMagnitude() <= 1.8)) {
                        setPathState();
                    }
                    break;


                case 7: // Return to Shoot 2
                    intake.doIntake();
                    if (!follower.isBusy()) {
                        follower.followPath(travelBackToShootFromGate, false);
                        setPathState();
                    }
                    break;


                case 8: // Shoot 3 Balls (Cycle 2)
                    stopIntakeOnceAtT(0.45);

                    // Shooter logic owns intake AFTER the stop
                    if (intakeStoppedForShooting) {
                        handleAutoShooting(currentPose, targetX, 4, 0,false);
                    }

                    // Allow feeding once fully settled
                    if (intakeStoppedForShooting
                            && !goForLaunch
                            && follower.atParametricEnd()
                            && follower.getVelocity().getMagnitude() < 1.3) {
                        goForLaunch = true;
                    }
                    break;

                case 9: // Shoot 3 Balls (Cycle 1)
                    if (!follower.isBusy()) {
                        intake.doIntake();
                        follower.followPath(intake3, false);
                        setPathState();
                    }
                    break;
                case 10:
                    intake.doIntake();
                    if (!follower.isBusy() || (follower.getVelocity().getMagnitude() <=1.4 && pathTimer.getElapsedTimeSeconds() > 0.7)) {
                        follower.followPath(travelBackToShootFromIntake3, false);
                        setPathState();
                    }
                    break;

                case 11: // Return to Shoot 1
                    stopIntakeOnceAtT(0.45);

                    // Shooter logic owns intake AFTER the stop
                    if (intakeStoppedForShooting) {
                        handleAutoShooting(currentPose, targetX, 4, 0,false);
                    }

                    // Allow feeding once fully settled
                    if (intakeStoppedForShooting
                            && !goForLaunch
                            && follower.atParametricEnd()
                            && follower.getVelocity().getMagnitude() < 1.3) {
                        goForLaunch = true;
                    }
                    break;

                case 12:
                    intake.doIntake();
                    if (!follower.isBusy()) {
                        follower.followPath(intake1, false);
                        setPathState();
                    }
                    break;

                case 13:
                    if (!follower.isBusy() || (follower.getVelocity().getMagnitude() <=1.4 && pathTimer.getElapsedTimeSeconds() > 0.7)) {
                        follower.followPath(travelBackToShootFromIntake1, false);
                        setPathState();
                    }
                    break;

                case 14: // Return to Shoot 1
                    stopIntakeOnceAtT(0.45);

                    // Shooter logic owns intake AFTER the stop
                    if (intakeStoppedForShooting) {
                        handleAutoShooting(currentPose, targetX, 25, 0,true);
                    }

                    // Allow feeding once fully settled
                    if (intakeStoppedForShooting
                            && !goForLaunch

                            && follower.getVelocity().getMagnitude() < 1) {
                        goForLaunch = true;
                    }
                    break;

                default:
                    shooter.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC,0,0);
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
                double mannualHoodOffset,boolean lastTime
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
                if (!lastTime) {
                    trackShotCount(shooter.isBeamBroken());
                }
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
            allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

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
                follower.setStartingPose(Poses.get(Poses.startPose15Ball));
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
            telemetry.addData("Start Pose", Poses.get(Poses.startPose15Ball));

            telemetry.addData("X Pos", follower.getPose().getX());
            telemetry.addData("Y Pos", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();

            lastBeamState = shooter.isBeamBroken();
        }

        @Override
        public void start() {
            pathTimer.resetTimer();
            setPathState(0);
        }

        @Override
        public void loop() {

            // Clear bulk cache ONCE per loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //loopTimer.resetTimer();

            follower.update();
            pinpoint.update();



            // Cache hardware reads ONCE
            boolean beamBroken = shooter.isBeamBroken();
            double flywheelVelo = shooter.getFlywheelVelo();
            double targetFlywheelVelo = shooter.getTargetFLywheelVelo();

            shooter.update(shooter.getCurrentTurretPosition());
            intake.update(
                    beamBroken,
                    LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE,
                    null, null, null
            );

            autonomousPathUpdate();

            veloReached =
                    (Math.abs(flywheelVelo) >
                            (Math.abs(targetFlywheelVelo) - 40)
                            &&
                            Math.abs(flywheelVelo) <
                                    (Math.abs(targetFlywheelVelo) + 40)
                            &&
                            Math.abs(targetFlywheelVelo) >= 1);

            telemetry.addData("State", pathState);
            telemetry.addData("Balls Fired", ballsShotInState);
            telemetry.addData("Beam Status", beamBroken ? "BROKEN" : "CLEAR");
            telemetry.addData("Shooter Velo", flywheelVelo);
            telemetry.addData("is up to sped", veloReached);
            telemetry.addData("Balls shot in state:", ballsShotInState);
            //telemetry.addData("Loop Time", loopTimer.getElapsedTime());
            telemetry.addData("target velo", targetFlywheelVelo);
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
    }

