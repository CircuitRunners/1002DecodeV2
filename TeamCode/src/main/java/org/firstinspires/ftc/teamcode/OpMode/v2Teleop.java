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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

@TeleOp(name = "Zenith Teleop", group = "A")
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
    private int opState = 0; // 0: Intake, 1: Sort, 2: Score
    private boolean isRedAlliance = true;
    private LimelightCamera.BallOrder targetPattern = null;

    // Field Constants
    private final double RED_GOAL_X = 132.0;
    private final double BLUE_GOAL_X = 12.0;
    private final double GOAL_Y = 137.0;
    private static final double METERS_TO_INCH = 39.37;

    @Override
    public void init() {
        // Initialize Localization (Pedro Pathing)
        follower = Constants.createFollower(hardwareMap);

        // Setup Pinpoint for high-speed velocity data
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint(); // Restore the config method call

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();

        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        player1 = new GamepadEx(gamepad1);

        telemetry.addLine("Zenith Ready.");
    }

    @Override
    public void init_loop() {
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) isRedAlliance = true;
        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) isRedAlliance = false;
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. HARDWARE UPDATES
        follower.update();
        pinpoint.update(); // Manual update for velocity reads
        sensors.update();
        player1.readButtons();

        Pose currentPose = follower.getPose();

        // 2. COORDINATE OVERRIDES
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(-90)));
            gamepad1.rumble(200);
        }

        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            updateCoordinatesWithAprilTag();
        }

        // 3. BUMPER STATE LOGIC
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (opState == 0) opState = 1;
            else if (opState == 1) opState = 2;
            else resetToIntake();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (opState != 2) opState = 2;
            else resetToIntake();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.B)) {
            resetToIntake();
        }

        // 4. EXECUTE MODES
        handleDriving(currentPose);

        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleSortingState(); break;
            case 2: handleScoringState(currentPose); break;
        }

        // 5. SUBSYSTEM SYNC
        intake.setCanShoot(shooter.flywheelVeloReached && shooter.turretReached && shooter.hoodReached);
        shooter.update(sensors.getFlywheelVelo(), sensors.getTurretPosition());

        doTelemetry(currentPose);
    }

    private void handleDriving(Pose pose) {
        double forward = -player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (isRedAlliance) {
            forward = -forward;
            strafe = -strafe;
        }

        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);

        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void handleIntakeState() {
        LimelightCamera.BallOrder seen = limelight.detectBallOrder();
        if (seen != null) targetPattern = seen;

        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.intake();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.retainBalls();
    }

    private void handleSortingState() {
        runSortingLogic();
    }

    private void handleScoringState(Pose pose) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;

        // VELOCITY COMPENSATION via Pinpoint
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);

        shooter.setShooterTarget(
                pose.getX(), pose.getY(),
                targetX, GOAL_Y,
                velX, velY,
                sensors.getTurretPosition(),
                Math.toDegrees(pose.getHeading()),
                Shooter.TurretMode.AUTO_ALIGN, 0
        );

        runSortingLogic();
    }

    private void resetToIntake() {
        opState = 0;
        shooter.stopFlywheel();
        intake.sortManualOverride();
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

    public void updateCoordinatesWithAprilTag() {
        limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    Pose3D mt1Pose = result.getBotpose();
                    if (mt1Pose != null) {
                        double llX_in = mt1Pose.getPosition().x * METERS_TO_INCH;
                        double llY_in = mt1Pose.getPosition().y * METERS_TO_INCH;

                        double llX_rotated = llY_in;
                        double llY_rotated = -llX_in;
                        double finalX = llX_rotated + 72.0;
                        double finalY = llY_rotated + 72.0;

                        follower.setPose(new Pose(finalX, finalY, follower.getHeading()));
                        gamepad1.rumble(500);
                    }
                    break;
                }
            }
        }
    }

    // --- PINPOINT CONFIGURATION (RESTORED) ---
    private void configurePinpoint() {
        pinpoint.setOffsets(44.94, -170.367, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    private void doTelemetry(Pose pose) {
        telemetry.addData("STATE", opState == 0 ? "INTAKE" : opState == 1 ? "SORTING" : "SCORING");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Pos", String.format(Locale.US, "%.1f, %.1f, %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        telemetry.addData("Ready", intake.canShoot ? "YES" : "WAITING");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.limelightCamera.stop();
    }
}