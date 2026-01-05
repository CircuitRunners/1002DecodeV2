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
// Assuming this is where your Poses class lives
// import org.firstinspires.ftc.teamcode.Config.Util.Poses;

@TeleOp(name = "Zenith Teleop V2 - Final", group = "A")
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

    private int opState = 0;
    private boolean isRedAlliance = true;
    private boolean preselectFromAuto = false;

    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    private final double RED_GOAL_X = 132.0;
    private final double BLUE_GOAL_X = 12.0;
    private final double GOAL_Y = 137.0;
    private static final double METERS_TO_INCH = 39.37;

    private final ElapsedTime timer = new ElapsedTime();

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

        // --- ALLIANCE PRE-SELECTION LOGIC ---
        // Replace 'Poses' with your actual class name if different
         if(Poses.getAlliance() != null){
            if(Poses.getAlliance() == Poses.Alliance.RED){
                isRedAlliance = true;
                preselectFromAuto = true;
            } else {
                isRedAlliance = false;
                preselectFromAuto = true;
            }
        } else {
            isRedAlliance = true; // Default to Red if null
            preselectFromAuto = false;
        }

    }

    @Override
    public void init_loop() {
        // Allow alliance switching during init_loop as well
        handleAllianceToggles();

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Preselect", preselectFromAuto ? "YES" : "NO (Manual)");
        telemetry.addData("Pattern", Intake.targetPatternFromAuto == null ? "NULL" : Intake.targetPatternFromAuto);
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        pinpoint.update();
        sensors.update();
        player1.readButtons();

        Pose currentPose = follower.getPose();
        Pose2D currentPinpointPose = pinpoint.getPosition();

        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );
        telemetry.addData("FOLLOWER (Pedro) Position", followerData);

//        String pinpointData = String.format(Locale.US,
//                "{X: %.3f, Y: %.3f, H: %.3f}",
//                currentPinpointPose.getX(DistanceUnit.INCH),
//                currentPinpointPose.getY(DistanceUnit.INCH),
//                currentPinpointPose.getHeading(AngleUnit.DEGREES)
//        );
//        // Pinpoint / Odo Raw
//        telemetry.addData("Pinpoint Position", pinpointData);

        handleAllianceToggles(); // Allow real-time changes
        handleInputOverrides();
        handleDriving(currentPose);

        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringStateNoSort(currentPose); break;
            case 2: handleScoringState(currentPose); break;
        }

        intake.setCanShoot(shooter.flywheelVeloReached && shooter.turretReached && shooter.hoodReached);
        shooter.update(sensors.getFlywheelVelo(), sensors.getSketchTurretPosition());

        doTelemetry();
        extraTelemetryForTesting(); // NEW: Diagnostics for testing

        telemetry.update();
        timer.reset();
    }

    private void handleAllianceToggles() {
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
            isRedAlliance = true;
            gamepad1.rumble(100);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) {
            isRedAlliance = false;
            gamepad1.rumble(100);
        }
    }

    private void handleInputOverrides() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(-90)));
        }

        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            updateCoordinatesWithAprilTag();
        }

        // Rescan Pattern
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            LimelightCamera.BallOrder seen = limelight.detectBallOrder();
            if (seen != null) {
                Intake.targetPatternFromAuto = seen;
                gamepad1.rumble(500);
            }
        }

        // Mode Toggles
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (opState != 1) { ballsShotInState = 0; opState = 1; }
            else {resetToIntake(); }
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (opState != 2) { ballsShotInState = 0; opState = 2; }
            else {resetToIntake(); }
        }
    }

    private void handleIntakeState() {
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.intake();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.retainBalls();
    }

    private void handleScoringStateNoSort(Pose pose) {
        applyShooterTargets(pose);
        if (shooter.flywheelVeloReached && shooter.hoodReached) {
            intake.transfer();
            trackShotCount();
        }
        if (ballsShotInState >= 3) resetToIntake();
    }

    private void handleScoringState(Pose pose) {
        applyShooterTargets(pose);

        // Safety check for null pattern
        LimelightCamera.BallOrder activePattern = (Intake.targetPatternFromAuto != null)
                ? Intake.targetPatternFromAuto
                : LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;

        intake.sort(sensors.getBeamBreakValue(), activePattern,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        trackShotCount();
        if (ballsShotInState >= 3) resetToIntake();
    }

    private void trackShotCount() {
        boolean currentBeamState = sensors.isBeamBroken();
        if (lastBeamState && !currentBeamState) {
            ballsShotInState++;
        }
        lastBeamState = currentBeamState;
    }

    private void resetToIntake() {
        opState = 0;
        ballsShotInState = 0;
        shooter.stopFlywheel();
        intake.sortManualOverride();
        intake.resetIndexer();
        gamepad1.rumble(150);
    }

    private void applyShooterTargets(Pose pose) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        shooter.setShooterTarget(pose.getX(), pose.getY(), targetX, GOAL_Y,
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH),
                Math.toDegrees(pose.getHeading()), true);
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
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }


    private void doTelemetry() {
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("MODE", opState == 0 ? "INTAKE" : opState == 1 ? "BURST" : "SORT");
        telemetry.addData("SHOTS", ballsShotInState + "/3");
        telemetry.addData("PATTERN", Intake.targetPatternFromAuto == null ? "DEFAULT (PPG)" : Intake.targetPatternFromAuto);
        telemetry.addData("Loop Time", timer.milliseconds());
    }

    private void extraTelemetryForTesting() {
        telemetry.addLine("--- DIAGNOSTICS ---");
        // Shooter Diagnostics
        telemetry.addData("Flywheel Reached", shooter.flywheelVeloReached);
        telemetry.addData("Hood Reached", shooter.hoodReached);
        telemetry.addData("Turret Reached", shooter.turretReached);
        telemetry.addData("Current Flywheel Velo", sensors.getFlywheelVelo());

        // Sensor/Intake Diagnostics
        telemetry.addData("Beam Break", sensors.isBeamBroken());
        telemetry.addData("Color 1 Detected", sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()));
        telemetry.addData("Color 2 Detected", sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()));
        telemetry.addData("Color 3 Detected", sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        // Raw Color Sensor Values (Red/Blue/Green)
        telemetry.addData("S1 Raw", "R:%d B:%d G:%d", sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green());
        telemetry.addData("S2 Raw", "R:%d B:%d G:%d", sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green());
        telemetry.addData("S3 Raw", "R:%d B:%d G:%d", sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green());

    }

    @Override public void stop() { limelight.limelightCamera.stop(); }
}