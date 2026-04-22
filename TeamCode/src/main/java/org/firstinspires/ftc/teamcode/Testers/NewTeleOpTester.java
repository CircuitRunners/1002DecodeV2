package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "New TeleOp Tester", group = "B")
@Configurable
public class NewTeleOpTester extends OpMode {

    private List<LynxModule> allHubs;

    private double mannualFlywheelAdj = 0;

    private MecanumDrive drive;


    private Intake intake;
    private NewShooter shooter;
    private Sensors sensors;
    private Follower follower;
    private GamepadEx player1;
    private GamepadEx player2;

    private int opState = 0;
    private boolean isRedAlliance = true;
    private boolean preselectFromAuto = false;

    public Limelight3A limelight67;

    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    private final double RED_GOAL_X = 130;
    private final double BLUE_GOAL_X = 7.5;
    private final double GOAL_Y = 132;

    private double dist = 0;
    private double newX = BLUE_GOAL_X;
    private double newY = GOAL_Y;

    private double newGoalCoords[];

    private double turretOffsetFar = 0;

    // --- Change 2: heading lag prediction fields ---
    private double prevHeadingDeg = 0;
    private final ElapsedTime headingDeltaTimer = new ElapsedTime();
    public static double headingPredictLagSec = 0.035;

    boolean veloReached = false;

    private boolean vibratedYet = false;
    private boolean initiateTransfer = false;

    public static boolean shootOnDaMove = true;

    private boolean noAutoAlign = false;
    boolean useAprilTagAim = false;
    int aprilTagCounter = 0;

    private boolean lastRightBumper = false;

    double llError = 0;

    // --- Heading correction from AprilTag ---
    public static double headingCorrCameraOffset = 0.0;   // tune if camera isn't centered on turret axis
    public static double headingCorrMaxAdjustDeg = 12.0;  // refuse snap if correction exceeds this
    public static double headingCorrVeloThreshold = 10.0; // inches/sec — only correct when roughly still

    private final ElapsedTime timer = new ElapsedTime();
    private double turretMannualAdjust = 0;

    boolean teleopShootApporval = false;

    public static double[] turretCoefficientsTeleop = {0.06, 0.00, 0.00225, 0.0024125};
    public static double limelightTurretScale = 1.0;
    public static double limelightTurretTolerance = 4.8;

    public static double limelightFarZoneOffset = -2.2;
    public static double limelightCloseZoneOffset = 1.5;

    public static double turretDeadband = 0;

    LLResult result = null;



    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());
        follower.update();

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new NewShooter(hardwareMap, telemetry, false);

        limelight67 = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight67.pipelineSwitch(3);
        limelight67.start();

        sensors = new Sensors();
        sensors.lightInit(hardwareMap);

        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);

        preselectFromAuto = Poses.getAlliance() != null;
        isRedAlliance = preselectFromAuto && (Poses.getAlliance() == Poses.Alliance.RED);

    }

    @Override
    public void init_loop() {
        handleAllianceToggles();
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Preselect", preselectFromAuto ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) hub.clearBulkCache();
        timer.reset();
        // --- 1. HARDWARE UPDATES ---
        follower.update();
        player1.readButtons();
        player2.readButtons();

        follower.getVelocity();

        veloReached = (Math.abs(shooter.getFlywheelVelo()) > (Math.abs(0.88 * shooter.getTargetFLywheelVelo())) && Math.abs(shooter.getFlywheelVelo()) < (Math.abs(1.09 * shooter.getTargetFLywheelVelo())) && Math.abs(shooter.getTargetFLywheelVelo()) >= 1);

        if (limelight67.getLatestResult().isValid() && llError <= 1.1) {
            sensors.setLight(0.600);
        } else {
            sensors.setLight(0.5);
        }

        // --- 2. DATA SNAPSHOTS ---
        Pose currentPose = follower.getPose();
        double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
        double robotVelX = follower.getVelocity().getXComponent();
        double robotVelY = follower.getVelocity().getYComponent();

        // --- Change 2: predictive heading correction for rotation lag ---
        double loopDt = headingDeltaTimer.seconds();
        headingDeltaTimer.reset();
        double angVelDegPerSec = (loopDt > 0.001)
                ? (normalizeAngleDiff(currentHeadingDeg - prevHeadingDeg) / loopDt)
                : 0;
        double predictedHeadingDeg = currentHeadingDeg + angVelDegPerSec * headingPredictLagSec;
        prevHeadingDeg = currentHeadingDeg;

        if (follower.getVelocity().getMagnitude() <= 20.0) { shootOnDaMove = false; }
        else { shootOnDaMove = true; }

        LimelightCamera.BallOrder activePattern = (Intake.targetPatternFromAuto != null)
                ? Intake.targetPatternFromAuto
                : LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;

        double currentFlywheelVelo = shooter.getFlywheelVelo();
        double currentTurretAngle = shooter.getCurrentTurretPosition();
        boolean isBeamBroken = shooter.isBeamBroken();

        result = limelight67.getLatestResult();

        if (result != null && result.isValid()) {
            llError = -result.getTy();
        } else {
            llError = 0;
        }

        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addLine("ZENITH TUFF AHH 67 67 CITY BOI [TESTER]");
        telemetry.addData("Position", followerData);
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("MODE", opState == 0 ? "INTAKE" : opState == 1 ? "BURST" : "SORT");
        telemetry.addData("Loop Time", "%.2f ms", timer.milliseconds());
        telemetry.addData("Flywheel Reached", veloReached);
        telemetry.addData("Turret Reached", shooter.turretReached ? "YEA" : "NAH");
        telemetry.addData("Turret Mode", useAprilTagAim ? "LIMELIGHT" : "FIELD");
        telemetry.addData("Limelight Counter", aprilTagCounter);
        telemetry.addData("Shoot While Moving", shootOnDaMove ? "YEA" : "NAH");
        telemetry.addData("Predicted Heading", "%.2f (raw: %.2f)", predictedHeadingDeg, currentHeadingDeg);
        telemetry.addData("Angular Vel (deg/s)", "%.1f", angVelDegPerSec);
        telemetry.addData("Tag Heading Correction", follower.getVelocity().getMagnitude() <= headingCorrVeloThreshold && result != null && result.isValid() ? "ACTIVE" : "waiting");

        telemetry.addLine("--- DIAGNOSTICS ---");
        telemetry.addData("Distance From Goal", Math.hypot(isRedAlliance ? Math.abs(RED_GOAL_X - currentPose.getX()) : Math.abs(BLUE_GOAL_X - currentPose.getX()), Math.abs(GOAL_Y - currentPose.getY())));
        telemetry.addData("Turret Ang", "%.2f", currentTurretAngle);
        telemetry.addData("DESIRED VELO:", shooter.getTargetFLywheelVelo());
        telemetry.addData("Flywheel Velo", currentFlywheelVelo);
        telemetry.addData("Shooter 1 Velo", shooter.shooter1.getVelocity());
        telemetry.addData("Shooter 2 Velo", shooter.shooter2.getVelocity());
        telemetry.addData("Beam Broken", isBeamBroken);
        telemetry.addData("Balls shot:", ballsShotInState);
        telemetry.addData("Turret Limelight Error", llError);

        // --- 3. LOGIC & OVERRIDES ---
        correctHeadingFromTag(currentPose);
        // If correctHeadingFromTag snapped the heading, reset prevHeadingDeg so the angular
        // velocity calculation next loop doesn't interpret the snap as a real rotation spike.
        double postCorrectionHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        if (Math.abs(normalizeAngleDiff(postCorrectionHeadingDeg - currentHeadingDeg)) > 0.5) {
            prevHeadingDeg = postCorrectionHeadingDeg;
        }
        handleManualTurretOverrides(follower.getPose().getHeading());
        handleAllianceToggles();
        handleInputOverrides();
        handleDriving(currentPose);

        // --- 4. STATE MACHINE ---
        // Change 2: pass predictedHeadingDeg instead of currentHeadingDeg
        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringStateNoSort(currentPose, robotVelX, robotVelY, predictedHeadingDeg, isBeamBroken); break;
            case 2: handleScoringState(currentPose, robotVelX, robotVelY, predictedHeadingDeg, isBeamBroken); break;
        }

        // --- 5. SUBSYSTEM UPDATES ---
        intake.setCanShoot(veloReached && shooter.turretReached && teleopShootApporval);
        shooter.update(currentTurretAngle);
        intake.update(isBeamBroken, activePattern, null, null, null);

        // --- 6. TELEMETRY ---
        telemetry.update();
    }

    private void handleManualTurretOverrides(double currentAngle) {
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            turretMannualAdjust += 5;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            turretMannualAdjust -= 5;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            mannualFlywheelAdj -= 3.5;
        }

        if (player2.wasJustPressed(GamepadKeys.Button.CROSS)) {
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            mannualFlywheelAdj += 3.5;
        }
    }

    private void handleScoringStateNoSort(Pose pose, double vx, double vy, double head, boolean beam) {

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            useAprilTagAim = !useAprilTagAim;
        }

        if (follower.getPose().getY() < 50 &&
                result != null &&
                result.isValid() &&
                !shootOnDaMove &&
                useAprilTagAim) {
            updateTurretWithAprilTag();
            noAutoAlign = true;
        } else {
            noAutoAlign = false;
        }
        applyShooterTargets(pose, vx, vy, head);

        if (veloReached && !vibratedYet) {
            gamepad1.rumble(250);
            vibratedYet = true;
        } else if (vibratedYet && (gamepad1.right_trigger > 0.2)) {
            initiateTransfer = true;
        }

        if (initiateTransfer) {
            trackShotCount(beam);
        }
        if (initiateTransfer && veloReached && (pose.getY() > 80)) {
            teleopShootApporval = true;
            intake.doTestShooter();
        } else if (initiateTransfer && veloReached && (pose.getY() <= 80 && gamepad1.right_trigger > 0.2)) {
            teleopShootApporval = true;
            intake.doTestShooter();
        } else {
            intake.doIntakeHalt();
        }
    }

    private void handleScoringState(Pose pose, double vx, double vy, double head, boolean beam) {
        applyShooterTargets(pose, vx, vy, head);

        if (veloReached && !vibratedYet) {
            gamepad1.rumble(250);
            vibratedYet = true;
        } else if (!intake.canShoot) {
            vibratedYet = false;
        }
        if (vibratedYet && (gamepad1.right_trigger > 0.2)) {
            initiateTransfer = true;
        }

        if (initiateTransfer) {
            teleopShootApporval = true;
            trackShotCount(beam);
        }

        if (ballsShotInState >= 3) resetToIntake();
    }

    private void applyShooterTargets(Pose pose, double vx, double vy, double headingDeg) {
        boolean closeZone = pose.getY() > 62;
        double targetX;
        if (isRedAlliance) {
            if (closeZone) {
                targetX = RED_GOAL_X - 2.5;
            } else {
                targetX = RED_GOAL_X - 5.5;
            }
        } else {
            // Change 3: removed heading-dependent targetX jump for blue
            targetX = BLUE_GOAL_X;
        }

        boolean autoAlign = !noAutoAlign;
        double rx = Math.round((pose.getX() * 10) / 10);
        double ry = Math.round((pose.getY() * 10) / 10);

        if (closeZone) {
            if (shootOnDaMove) {
                newGoalCoords = shooter.computeVelocityCompensatedPositionFirestorm(targetX, GOAL_Y, pose.getX(), pose.getY(), follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
                newX = newGoalCoords[0];
                newY = newGoalCoords[1];
                dist = Math.hypot(newX - pose.getX(), newY - pose.getY());
                shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), isRedAlliance ? newX - 2.5 : newX + 2, newY, headingDeg, true, isRedAlliance ? -38 : -15, 0, false, 0);
                if (gamepad1.right_trigger > 0.2) intake.doTestShooter();
            } else {
                dist = Math.hypot(isRedAlliance ? (targetX - 2.5) - pose.getX() : targetX - pose.getX(), GOAL_Y - pose.getY());
                newX = targetX;
                newY = GOAL_Y;
                shooter.setTargetsByDistanceAdjustable(rx, ry, targetX, GOAL_Y, headingDeg, autoAlign, isRedAlliance ? -31 : -15, 0, isRedAlliance, turretMannualAdjust);
            }
        } else {
            if (shootOnDaMove) {
                newGoalCoords = shooter.computeVelocityCompensatedPositionFirestorm(targetX, GOAL_Y, pose.getX(), pose.getY(), follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
                newX = newGoalCoords[0];
                newY = newGoalCoords[1];
                dist = Math.hypot(newX - pose.getX(), newY - pose.getY());
                shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), newX + 2, newY, headingDeg, true, -45, 0, false, 0);
                if (gamepad1.right_trigger > 0.2) intake.doTestShooter();
            } else {
                dist = Math.hypot(targetX - pose.getX(), GOAL_Y - pose.getY());
                newX = targetX;
                newY = GOAL_Y;
                shooter.setTargetsByDistanceAdjustable(rx, ry, targetX + 2, GOAL_Y, headingDeg, autoAlign, -50, 0, isRedAlliance, turretMannualAdjust);
            }
        }
    }

    private void handleDriving(Pose pose) {
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void trackShotCount(boolean currentBeamState) {
        if (lastBeamState && !currentBeamState) {
            ballsShotInState++;
        }
        lastBeamState = currentBeamState;
    }

    private void resetToIntake() {
        opState = 0;
        ballsShotInState = 0;
        teleopShootApporval = false;
        initiateTransfer = false;
        vibratedYet = false;
        shooter.stopFlywheel();
        intake.resetState();
        gamepad1.rumble(150);
    }

    private void handleAllianceToggles() {
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) {
            isRedAlliance = !isRedAlliance;
            gamepad1.rumble(100);
        }
    }

    private void handleInputOverrides() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            if (isRedAlliance) {
                follower.setPose(new Pose(0, 17, Math.toRadians(180)));
            } else {
                follower.setPose(new Pose(124, 9, Math.toRadians(0)));
            }
        }
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
        }
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (opState == 0) { ballsShotInState = 0; opState = 1; } else resetToIntake();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
            noAutoAlign = true;
        }

        if (gamepad1.left_trigger > 0.2) {
            intake.doOuttake();
        }
    }

    private void handleIntakeState() {
        if (gamepad1.right_trigger > 0.2) {
            intake.doIntake();
            if (!noAutoAlign) {
                shooter.setTurretTarget(0, NewShooter.TurretMode.ROBOT_CENTRIC, follower.getPose().getHeading(), turretMannualAdjust);
            }
        } else intake.doIntakeHalt();
    }

    public void updateTurretWithAprilTag() {
        Poses.Alliance alliance = Poses.getAlliance();
        if (alliance != null &&
                result != null &&
                result.isValid() &&
                (alliance == Poses.Alliance.RED ? getTagId() == 24 : getTagId() == 20))
        {
            double error = (-result.getTy());
            if (Math.abs(error) < limelightTurretTolerance) return;
            double currentTurretAngle = shooter.getCurrentTurretPosition();
            // Change 1: removed double-accumulation of currentTurretAngle
            double newTarget = currentTurretAngle + (error * limelightTurretScale);

            if (follower.getPose().getY() < 50) {
                shooter.setTurretTargetPosition(newTarget + limelightFarZoneOffset);
            } else {
                shooter.setTurretTargetPosition(newTarget + limelightCloseZoneOffset);
            }
            aprilTagCounter += 1;
        }
    }

    private void correctHeadingFromTag(Pose currentPose) {
        if (result == null || !result.isValid() || !useAprilTagAim) return;

        int expectedId = isRedAlliance ? 24 : 20;
        if (getTagId() != expectedId) return;

        if (follower.getVelocity().getMagnitude() > headingCorrVeloThreshold) return;

        double tagX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        double tagY = GOAL_Y;

        // Field angle to tag in CCW convention (standard atan2, degrees)
        double fieldAngleToTagCCW = Math.toDegrees(
                Math.atan2(tagY - currentPose.getY(), tagX - currentPose.getX())
        );

        // Turret angle in CCW convention (physical, per NewShooter comment)
        double turretAngleCCW = shooter.getCurrentTurretPosition();

        // ty: positive = target is physically right of boresight = CW offset from boresight
        // Adding ty cancels the CW offset when computing CCW heading
        double trueHeadingDeg = fieldAngleToTagCCW - turretAngleCCW - headingCorrCameraOffset + result.getTy();

        // Normalize to [0, 360)
        trueHeadingDeg = (trueHeadingDeg % 360 + 360) % 360;

        double currentHeadingDegLocal = (Math.toDegrees(currentPose.getHeading()) % 360 + 360) % 360;
        double diff = normalizeAngleDiff(trueHeadingDeg - currentHeadingDegLocal);

        // Safety: ignore implausibly large corrections (bad tag read, wrong tag, etc.)
        if (Math.abs(diff) > headingCorrMaxAdjustDeg) return;

        follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(trueHeadingDeg)));
    }

    // Change 2: helper to correctly handle heading wraparound for angular velocity
    private static double normalizeAngleDiff(double diff) {
        diff = diff % 360;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }

    @Override
    public void stop() {
    }

    public int getTagId() {
        if (result != null) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                return fiducial.getFiducialId();
            }
        }
        return 0;
    }
}
