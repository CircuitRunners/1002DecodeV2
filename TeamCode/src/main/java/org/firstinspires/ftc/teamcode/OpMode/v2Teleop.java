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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

@TeleOp(name = "Zenith Teleop V2 - Final", group = "A")
@Configurable
public class v2Teleop extends OpMode {

    // Subsystems
    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private LimelightCamera limelight;
    private Sensors sensors;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;

    private GamepadEx player1;

    // Logic States
    private int opState = 0; // 0: Intake, 1: Fast Burst (No Sort), 2: Smart Score (Sort)
    private boolean isRedAlliance = true;
    private LimelightCamera.BallOrder targetPattern = null;

    // Shot Counting & Beam Break Logic
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    // Constants
    private final double RED_GOAL_X = 132.0;
    private final double BLUE_GOAL_X = 12.0;
    private final double GOAL_Y = 137.0;
    private static final double METERS_TO_INCH = 39.37;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
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
        telemetry.addLine("Zenith Systems Online.");
    }

    @Override
    public void init_loop() {
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) isRedAlliance = true;
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) isRedAlliance = false;

        sensors.update();
        lastBeamState = sensors.isBeamBroken();

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. URGENT HARDWARE UPDATES
        follower.update();
        pinpoint.update();
        sensors.update();
        player1.readButtons();

        Pose currentPose = follower.getPose();

        // 2. INPUT MAPPING
        handleInputOverrides();

        // 3. STATE MACHINE EXECUTION
        handleDriving(currentPose);

        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringStateNoSort(currentPose); break;
            case 2: handleScoringState(currentPose); break;
        }

        // 4. SUBSYSTEM SYNC
        // Ensures the Intake subsystem's internal 'canShoot' logic respects Shooter readiness
        intake.setCanShoot(shooter.flywheelVeloReached && shooter.turretReached && shooter.hoodReached);

        // Shooter needs real-time sensor data for its internal PID loops
        shooter.update(sensors.getFlywheelVelo(), sensors.getSketchTurretPosition());

        doTelemetry(currentPose);
    }

    private void handleInputOverrides() {
        // Reset Odo (Square)
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(-90)));
            gamepad1.rumble(200);
        }

        // Relocalize (Triangle)
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            updateCoordinatesWithAprilTag();
        }

        // Manual Breakout (Circle)
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            resetToIntake();
            gamepad1.rumble(500);
        }

        // Scoring Mode Toggles
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { // L1 - Burst
            if (opState != 1) { ballsShotInState = 0; opState = 1; }
            else resetToIntake();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // R1 - Smart Sort
            if (opState != 2) { ballsShotInState = 0; opState = 2; }
            else resetToIntake();
        }
    }

    private void handleIntakeState() {
        LimelightCamera.BallOrder seen = limelight.detectBallOrder();
        if (seen != null) targetPattern = seen;

        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.intake();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.retainBalls();
    }

    private void handleScoringStateNoSort(Pose pose) {
        applyShooterTargets(pose);

        if (shooter.flywheelVeloReached && shooter.hoodReached) {
            intake.transfer(); // Use transfer to bypass front rollers
            trackShotCount();
        }

        if (ballsShotInState >= 3) {
            gamepad1.rumble(350); // Single long pulse
            resetToIntake();
        }
    }

    private void handleScoringState(Pose pose) {
        applyShooterTargets(pose);
        runSortingLogic(); // Uses Intake.sort() which uses colors/limelight
        trackShotCount();

        if (ballsShotInState >= 3) {
            gamepad1.rumble(150, 150, 300); // Triple short pulses
            resetToIntake();
        }
    }

    private void trackShotCount() {
        boolean currentBeamState = sensors.isBeamBroken();
        // Falling edge: broken -> not broken (ball has cleared the sensor)
        if (lastBeamState && !currentBeamState) {
            ballsShotInState++;
        }
        lastBeamState = currentBeamState;
    }

    private void applyShooterTargets(Pose pose) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        shooter.setShooterTarget(
                pose.getX(), pose.getY(), targetX, GOAL_Y,
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH),
                 Math.toDegrees(pose.getHeading()),
                true
        );
    }

    private void resetToIntake() {
        opState = 0;
        ballsShotInState = 0;
        shooter.stopFlywheel();
        intake.sortManualOverride(); // Critical: releases internal sort loops
        intake.resetIndexer();       // Critical: clears indexer flags
    }

    private void runSortingLogic() {
        intake.sort(
                sensors.getBeamBreakValue(),
                targetPattern,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green())
        );
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(44.94, -170.367, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    public void updateCoordinatesWithAprilTag() {
        limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    Pose3D mt1Pose = result.getBotpose();
                    if (mt1Pose != null) {
                        double finalX = (mt1Pose.getPosition().y * METERS_TO_INCH) + 72.0;
                        double finalY = (-mt1Pose.getPosition().x * METERS_TO_INCH) + 72.0;
                        follower.setPose(new Pose(finalX, finalY, follower.getHeading()));
                        gamepad1.rumble(500);
                    }
                    break;
                }
            }
        }
    }

    private void handleDriving(Pose pose) {
        double forward =-player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();
        if (isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void doTelemetry(Pose pose) {
        telemetry.addData("MODE", opState == 0 ? "INTAKE" : opState == 1 ? "BURST" : "SORT");
        telemetry.addData("SHOTS", ballsShotInState + "/3");
        telemetry.addData("HEADING", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    @Override public void stop() { limelight.limelightCamera.stop(); }
}