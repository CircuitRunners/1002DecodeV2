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

@TeleOp(name = "Zenith Teleop V2 - PS5", group = "A")
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
    private final ElapsedTime timer = new ElapsedTime();

    // Logic States
    private int opState = 0; // 0: Intake, 1: Fast Burst (No Sort), 2: Smart Score (Sort)
    private boolean isRedAlliance = true;
    private LimelightCamera.BallOrder targetPattern = null;

    // Shot Counting
    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    // Field Constants
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
        telemetry.addLine("Zenith PS5 Ready.");
    }

    @Override
    public void init_loop() {
        // Options = Red, Share = Blue
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) isRedAlliance = true;
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) isRedAlliance = false;
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. HARDWARE UPDATES
        follower.update();
        pinpoint.update();
        sensors.update();
        player1.readButtons();

        Pose currentPose = follower.getPose();

        // 2. INPUT HANDLING (PS5 Mapping)
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(-90)));
            gamepad1.rumble(200);
        }

        // Manual Breakout - Circle Button
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            resetToIntake();
            gamepad1.rumble(500);
        }

        // Mode Switching
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { // L1
            if (opState != 1) {
                ballsShotInState = 0;
                opState = 1;
            } else resetToIntake();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // R1
            if (opState != 2) {
                ballsShotInState = 0;
                opState = 2;
            } else resetToIntake();
        }

        // 3. EXECUTION
        handleDriving(currentPose);

        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringStateNoSort(currentPose); break;
            case 2: handleScoringState(currentPose); break;
        }

        // 4. SUBSYSTEM SYNC
        intake.setCanShoot(shooter.flywheelVeloReached && shooter.turretReached && shooter.hoodReached);
        shooter.update(sensors.getFlywheelVelo(), sensors.getSketchTurretPosition());

        doTelemetry(currentPose);
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
            intake.intake();
            trackShotCount();
        }

        if (ballsShotInState >= 3) {
            gamepad1.rumble(300);
            resetToIntake();
        }
    }

    private void handleScoringState(Pose pose) {
        applyShooterTargets(pose);
        runSortingLogic();
        trackShotCount();

        if (ballsShotInState >= 3) {
            gamepad1.rumble(100, 100, 200);
            resetToIntake();
        }
    }

    private void trackShotCount() {
        boolean currentBeamState = sensors.isBeamBroken();
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
                sensors.getSketchTurretPosition(), Math.toDegrees(pose.getHeading()),
                Shooter.TurretMode.AUTO_ALIGN, 0
        );
    }

    private void resetToIntake() {
        opState = 0;
        ballsShotInState = 0;
        shooter.stopFlywheel();
        intake.sortManualOverride();
        intake.resetIndexer();
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

    private void doTelemetry(Pose pose) {
        String modeName = (opState == 0) ? "INTAKE" : (opState == 1) ? "BURST (NO SORT)" : "SMART SCORE (SORT)";
        telemetry.addData("MODE", modeName);
        telemetry.addData("SHOTS", ballsShotInState + "/3");
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("POS", String.format(Locale.US, "%.1f, %.1f, %.1f°", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        telemetry.update();
    }

    private void handleDriving(Pose pose) {
        double forward = -player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();
        if (isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    @Override public void stop() { limelight.limelightCamera.stop(); }
}