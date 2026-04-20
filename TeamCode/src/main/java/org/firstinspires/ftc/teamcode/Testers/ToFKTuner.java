package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

/**
 * Full teleop with ToF K diagnostics.
 * Tune NewShooter.TOF_K via the Configurables dashboard panel — it's a public
 * static so it appears automatically. Drive/intake/shoot normally, then read
 * the "ToF K DIAGNOSTICS" block on telemetry to see what physicsModelTimeofFlight()
 * is predicting and compare it to your real-world observations.
 */
@Configurable
@TeleOp(name = "ToF K Tuner Teleop", group = "TEST")
public class ToFKTuner extends OpMode {

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
    private final double BLUE_GOAL_X = 9.5;
    private final double GOAL_Y = 132;
    public static double TOF_K = 0.45;

    private double dist = 0;
    private double newX;
    private double newY;
    private double[] newGoalCoords;

    boolean veloReached = false;

    private boolean vibratedYet = false;
    private boolean initiateTransfer = false;

    public static boolean shootOnDaMove = true;

    private boolean noAutoAlign = false;
    boolean useAprilTagAim = true;

    double llError = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private double turretMannualAdjust = 0;

    boolean teleopShootApporval = false;

    public static double[] turretCoefficientsTeleop = {0.06, 0.00, 0.00225, 0.0024125};
    public static double limelightTurretScale = 1.0;
    public static double limelightTurretTolerance = 4.8;
    public static double limelightFarZoneOffset = -2.2;
    public static double limelightCloseZoneOffset = +1.5;
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

        newX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        newY = GOAL_Y;
    }

    @Override
    public void init_loop() {
        handleAllianceToggles();
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("TOF_K (tune via dashboard)", "%.4f");
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        timer.reset();

        follower.update();
        player1.readButtons();
        player2.readButtons();

        veloReached = Math.abs(shooter.getFlywheelVelo()) > Math.abs(0.88 * shooter.getTargetFLywheelVelo())
                && Math.abs(shooter.getFlywheelVelo()) < Math.abs(1.09 * shooter.getTargetFLywheelVelo())
                && Math.abs(shooter.getTargetFLywheelVelo()) >= 1;

        if (limelight67.getLatestResult().isValid() && llError <= 1.1) {
            sensors.setLight(0.600);
        } else {
            sensors.setLight(0.5);
        }

        Pose currentPose = follower.getPose();
        double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
        double robotVelX = follower.getVelocity().getXComponent();
        double robotVelY = follower.getVelocity().getYComponent();

        LimelightCamera.BallOrder activePattern = (Intake.targetPatternFromAuto != null)
                ? Intake.targetPatternFromAuto
                : LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;

        double currentFlywheelVelo = shooter.getFlywheelVelo();
        double currentTurretAngle = shooter.getCurrentTurretPosition();
        boolean isBeamBroken = shooter.isBeamBroken();

        result = limelight67.getLatestResult();
        llError = (result != null && result.isValid()) ? -result.getTy() : 0;

        // --- ToF K diagnostics ---
        double hoodAngleRad = Math.toRadians(shooter.getCurrentRequiredHoodAngle());
        double predictedToF_Physics = shooter.physicsModelTimeofFlight(dist, currentFlywheelVelo, hoodAngleRad, TOF_K);
        double predictedToF_LUT = NewShooter.getShottimeFromDistanceLUT(dist);
        double ticksPerRev = 384.5;
        double flywheelDiam = 2.83;
        double surfaceSpeed = (currentFlywheelVelo / ticksPerRev) * Math.PI * flywheelDiam;
        double ballVelo = surfaceSpeed * TOF_K;

        // --- State machine ---
        handleManualTurretOverrides(currentPose.getHeading());
        handleAllianceToggles();
        handleInputOverrides();
        handleDriving(currentPose);

        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringState(currentPose, robotVelX, robotVelY, currentHeadingDeg, isBeamBroken); break;
        }

        intake.setCanShoot(veloReached && shooter.turretReached && teleopShootApporval);
        shooter.update(currentTurretAngle);
        intake.update(isBeamBroken, activePattern, null, null, null);

        // --- Telemetry ---
        String followerData = String.format(Locale.US, "{X:%.2f Y:%.2f H:%.1f}",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("=== ToF K TUNER TELEOP ===");
        telemetry.addData("Position", followerData);
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Mode", opState == 0 ? "INTAKE" : "SHOOT");
        telemetry.addData("Loop Time", "%.2f ms", timer.milliseconds());

        telemetry.addLine("--- ToF K DIAGNOSTICS ---");
        telemetry.addData("TOF_K  (tune in dashboard)", "%.4f", TOF_K);
        telemetry.addData("Hood Angle", "%.1f deg", shooter.getCurrentRequiredHoodAngle());
        telemetry.addData("Distance to goal", "%.1f in", dist);
        telemetry.addData("Flywheel surface speed", "%.1f in/s", surfaceSpeed);
        telemetry.addData("Ball velocity (K applied)", "%.1f in/s", ballVelo);
        telemetry.addData("Predicted ToF", (dist > 0.1 && currentFlywheelVelo > 100)
                ? String.format("%.4f s", predictedToF_Physics) : "N/A (not shooting)");

        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("Flywheel target", "%.0f", shooter.getTargetFLywheelVelo());
        telemetry.addData("Flywheel actual", "%.0f", currentFlywheelVelo);
        telemetry.addData("Velo reached", veloReached);
        telemetry.addData("Turret reached", shooter.turretReached);
        telemetry.addData("Beam broken", isBeamBroken);
        telemetry.addData("Balls shot", ballsShotInState);

        telemetry.update();
    }

    private void handleScoringState(Pose pose, double vx, double vy, double head, boolean beam) {
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            useAprilTagAim = !useAprilTagAim;
        }

        if (follower.getPose().getY() < 69 && result != null && result.isValid() && useAprilTagAim) {
            updateTurretWithAprilTag();
            noAutoAlign = true;
        } else {
            noAutoAlign = false;
        }

        applyShooterTargets(pose, vx, vy, head);

        if (veloReached && !vibratedYet) {
            gamepad1.rumble(250);
            vibratedYet = true;
        } else if (vibratedYet && gamepad1.right_trigger > 0.2) {
            initiateTransfer = true;
        }

        if (initiateTransfer) trackShotCount(beam);

        if (initiateTransfer && veloReached && pose.getY() > 80) {
            teleopShootApporval = true;
            intake.doTestShooter();
        } else if (initiateTransfer && veloReached && pose.getY() <= 80 && gamepad1.right_trigger > 0.2) {
            teleopShootApporval = true;
            intake.doTestShooter();
        } else {
            intake.doIntakeHalt();
        }
    }

    private void applyShooterTargets(Pose pose, double vx, double vy, double headingDeg) {
        boolean closeZone = pose.getY() > 62;
        double targetX;
        if (isRedAlliance) {
            targetX = closeZone ? RED_GOAL_X - 2.5 : RED_GOAL_X - 7.5;
        } else {
            boolean headingComp = headingDeg >= 345 || headingDeg <= 15;
            targetX = closeZone ? BLUE_GOAL_X : (headingComp ? BLUE_GOAL_X - 5 : BLUE_GOAL_X);
        }

        boolean autoAlign = !noAutoAlign;
        double rx = Math.round((pose.getX() * 10) / 10.0);
        double ry = Math.round((pose.getY() * 10) / 10.0);

        if (closeZone) {
            if (shootOnDaMove) {
                newGoalCoords = shooter.computeVelocityCompensatedPositionFirestorm(
                        targetX, GOAL_Y, pose.getX(), pose.getY(), vx, vy);
                newX = newGoalCoords[0];
                newY = newGoalCoords[1];
                dist = Math.hypot(newX - pose.getX(), newY - pose.getY());
                shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(),
                        isRedAlliance ? newX - 2.5 : newX + 2, newY,
                        headingDeg, true, isRedAlliance ? -38 : -15, 0, false, 0);
                if (gamepad1.right_trigger > 0.2) intake.doTestShooter();
            } else {
                dist = Math.hypot(isRedAlliance ? (targetX - 2.5) - pose.getX() : targetX - pose.getX(),
                        GOAL_Y - pose.getY());
                newX = targetX;
                newY = GOAL_Y;
                shooter.setTargetsByDistanceAdjustable(rx, ry, targetX, GOAL_Y, headingDeg, autoAlign,
                        isRedAlliance ? -31 : -15, 0, isRedAlliance, turretMannualAdjust);
            }
        } else {
            if (shootOnDaMove) {
                newGoalCoords = shooter.computeVelocityCompensatedPositionFirestorm(
                        targetX, GOAL_Y, pose.getX(), pose.getY(), vx, vy);
                newX = newGoalCoords[0];
                newY = newGoalCoords[1];
                dist = Math.hypot(newX - pose.getX(), newY - pose.getY());
                shooter.setTargetsByDistanceAdjustable(pose.getX(), pose.getY(), newX + 2, newY,
                        headingDeg, true, -45, 0, false, 0);
                if (gamepad1.right_trigger > 0.2) intake.doTestShooter();
            } else {
                dist = Math.hypot(targetX - pose.getX(), GOAL_Y - pose.getY());
                newX = targetX;
                newY = GOAL_Y;
                shooter.setTargetsByDistanceAdjustable(rx, ry, targetX + 2, GOAL_Y, headingDeg, autoAlign,
                        -50, 0, isRedAlliance, turretMannualAdjust);
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

    private void handleIntakeState() {
        if (gamepad1.right_trigger > 0.2) {
            intake.doIntake();
            if (!noAutoAlign) {
                shooter.setTurretTarget(0, NewShooter.TurretMode.ROBOT_CENTRIC,
                        follower.getPose().getHeading(), turretMannualAdjust);
            }
        } else {
            intake.doIntakeHalt();
        }
    }

    private void handleManualTurretOverrides(double currentAngle) {
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) turretMannualAdjust += 5;
        else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) turretMannualAdjust -= 5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) mannualFlywheelAdj -= 3.5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) mannualFlywheelAdj += 3.5;
    }

    private void handleAllianceToggles() {
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) {
            isRedAlliance = !isRedAlliance;
            gamepad1.rumble(100);
        }
    }

    private void handleInputOverrides() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            follower.setPose(isRedAlliance
                    ? new Pose(0, 17, Math.toRadians(180))
                    : new Pose(124, 9, Math.toRadians(0)));
        }
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (opState == 0) { ballsShotInState = 0; opState = 1; }
            else resetToIntake();
        }
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) noAutoAlign = true;
    }

    private void trackShotCount(boolean currentBeamState) {
        if (lastBeamState && !currentBeamState) ballsShotInState++;
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

    public void updateTurretWithAprilTag() {
        Poses.Alliance alliance = Poses.getAlliance();
        if (alliance == null || result == null || !result.isValid()) return;
        if (alliance == Poses.Alliance.RED ? getTagId() != 24 : getTagId() != 20) return;

        double error = -result.getTy();
        if (Math.abs(error) < limelightTurretTolerance) return;
        double current = shooter.getCurrentTurretPosition();
        double offset = follower.getPose().getY() < 50 ? limelightFarZoneOffset : limelightCloseZoneOffset;
        shooter.setTurretTargetPosition(current + error + offset);
    }

    public int getTagId() {
        if (result != null) {
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) return f.getFiducialId();
        }
        return 0;
    }

    @Override
    public void stop() {}
}
