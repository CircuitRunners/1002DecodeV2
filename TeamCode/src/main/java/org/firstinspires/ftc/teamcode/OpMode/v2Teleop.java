package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

@TeleOp(name = "Zenith Teleop V2 - Optimized", group = "A")
@Configurable
public class v2Teleop extends OpMode {

    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private LimelightCamera limelight;
    private Sensors sensors;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private GamepadEx player1;
    private GamepadEx player2;

    private int opState = 0;
    private boolean isRedAlliance = true;
    private boolean preselectFromAuto = false;

    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    private final double RED_GOAL_X = 132.0;
    private final double BLUE_GOAL_X = 12.0;
    private final double GOAL_Y = 137.0;
    private static final double METERS_TO_INCH = 39.37;

    private boolean vibratedYet = false;
    private boolean initiateTransfer = false;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());
        follower.update();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();

        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);

        if(Poses.getAlliance() != null){
            isRedAlliance = (Poses.getAlliance() == Poses.Alliance.RED);
            preselectFromAuto = true;
        } else {
            isRedAlliance = true;
            preselectFromAuto = false;
        }
    }

    @Override
    public void init_loop() {
        handleAllianceToggles();
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Preselect", preselectFromAuto ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- 1. HARDWARE UPDATES ---
        follower.update();
        pinpoint.update();
        sensors.update();
        player1.readButtons();
        player2.readButtons();

        // --- 2. DATA SNAPSHOTS (Call once, reference variables) ---
        Pose currentPose = follower.getPose();
        double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
        double robotVelX = pinpoint.getVelX(DistanceUnit.INCH);
        double robotVelY = pinpoint.getVelY(DistanceUnit.INCH);

        double currentFlywheelVelo = sensors.getFlywheelVelo();
        double currentTurretAngle = sensors.getSketchTurretPosition();
        boolean isBeamBroken = sensors.isBeamBroken();
        double beamValue = sensors.getBeamBreakValue();

        // --- 3. LOGIC & OVERRIDES ---
        handleManualTurretOverrides(currentTurretAngle);
        handleAllianceToggles();
        handleInputOverrides();
        handleDriving(currentPose);

        // --- 4. STATE MACHINE ---
        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringStateNoSort(currentPose, robotVelX, robotVelY, currentHeadingDeg, isBeamBroken); break;
            case 2: handleScoringState(currentPose, robotVelX, robotVelY, currentHeadingDeg, isBeamBroken, beamValue); break;
        }

        // --- 5. SUBSYSTEM UPDATES ---
        intake.setCanShoot(shooter.flywheelVeloReached && shooter.turretReached && shooter.hoodReached);
        shooter.update(currentFlywheelVelo, currentTurretAngle);

        // --- 6. TELEMETRY ---
        doTelemetry();
        extraTelemetryForTesting(currentFlywheelVelo, currentTurretAngle, isBeamBroken);
        telemetry.update();
        timer.reset();
    }

    private void handleManualTurretOverrides(double currentAngle) {
        // Manual control: move turret and stick PID to current position to prevent fighting
        if (gamepad2.right_bumper) {
            shooter.manualTurretOverride(0.5, currentAngle);
        } else if (gamepad2.left_bumper) {
            shooter.manualTurretOverride(-0.5, currentAngle);
        }

        // Hardware re-zero
        if (player2.wasJustPressed(GamepadKeys.Button.START)) {
            sensors.rezeroTurretEncoder();
            gamepad2.rumble(500);
        }
    }

    private void handleScoringStateNoSort(Pose pose, double vx, double vy, double head, boolean beam) {
        applyShooterTargets(pose, vx, vy, head);
        if (shooter.flywheelVeloReached && shooter.hoodReached && !vibratedYet) {
            gamepad1.rumble(250);
            vibratedYet = true;
        }
        else if (!shooter.flywheelVeloReached || !shooter.hoodReached){
            vibratedYet = false;
        }
        if (vibratedYet && (gamepad1.right_trigger > 0.2)){
            initiateTransfer = true;
        }

        if (initiateTransfer){
            intake.transfer();
            trackShotCount(beam);
        }

        if (ballsShotInState >= 3) resetToIntake();
    }

    private void handleScoringState(Pose pose, double vx, double vy, double head, boolean beam, double beamVal) {
        applyShooterTargets(pose, vx, vy, head);

        LimelightCamera.BallOrder activePattern = (Intake.targetPatternFromAuto != null)
                ? Intake.targetPatternFromAuto
                : LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;

        intake.sort(beamVal, activePattern,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        trackShotCount(beam);
        if (ballsShotInState >= 3) resetToIntake();
    }

    private void applyShooterTargets(Pose pose, double vx, double vy, double headingDeg) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        shooter.setShooterTarget(pose.getX(), pose.getY(), targetX, GOAL_Y, vx, vy, headingDeg, false); // TRUE for auto align
    }

    private void handleDriving(Pose pose) {
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (isRedAlliance) { forward = -forward; strafe = -strafe; }
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
        initiateTransfer = false;
        vibratedYet = false;
        shooter.stopFlywheel();
        intake.sortManualOverride();
        intake.resetIndexer();
        gamepad1.rumble(150);
    }

    private void handleAllianceToggles() {
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) { isRedAlliance = true; gamepad1.rumble(100); }
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) { isRedAlliance = false; gamepad1.rumble(100); }
    }

    private void handleInputOverrides() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(new Pose(72, 72, Math.toRadians(-90)));
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) updateCoordinatesWithAprilTag();
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            LimelightCamera.BallOrder seen = limelight.detectBallOrder();
            if (seen != null) { Intake.targetPatternFromAuto = seen; gamepad1.rumble(500); }
        }
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (opState != 1) { ballsShotInState = 0; opState = 1; } else resetToIntake();
        }
        /* not till tuned sry lil bro
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (opState != 2) { ballsShotInState = 0; opState = 2; } else resetToIntake();
        }
        */

    }

    private void handleIntakeState() {
        if (gamepad1.right_trigger > 0.2) intake.intake();
        else if (gamepad1.left_trigger > 0.2) intake.outtake();
        else intake.intakeMotorIdle();
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(44.94, -170.367, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    private void doTelemetry() {
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("MODE", opState == 0 ? "INTAKE" : opState == 1 ? "BURST" : "SORT");
        telemetry.addData("Loop Time", "%.2f ms", timer.milliseconds());
    }

    private void extraTelemetryForTesting(double fVelo, double tAng, boolean beam) {
        telemetry.addLine("--- DIAGNOSTICS ---");
        telemetry.addData("Turret Ang", "%.2f", tAng);
        telemetry.addData("Flywheel Velo", fVelo);
        telemetry.addData("Beam Broken", beam);
        telemetry.addData("Shot Possible", !shooter.isShotImpossible);
    }

    public void updateCoordinatesWithAprilTag() {
        limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();
        if (result != null && result.isValid()) {
            Pose3D mt1Pose = result.getBotpose();
            if (mt1Pose != null) {
                double finalX = (mt1Pose.getPosition().y * METERS_TO_INCH) + 72.0;
                double finalY = (-mt1Pose.getPosition().x * METERS_TO_INCH) + 72.0;
                follower.setPose(new Pose(finalX, finalY, follower.getHeading()));
                gamepad1.rumble(500);
            }
        }
    }

    @Override public void stop() { limelight.limelightCamera.stop(); }
}