package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "Shooter: K-Tuner v2.1", group = "TEST")
public class ShooterKTuner extends OpMode {

    // ===== Physics Constants (Standardized for 10ft Test) =====
    public static double TEST_DISTANCE_INCHES = 120.0;
 //   public static double GOAL_HEIGHT = 38.75;
    public static double LAUNCH_HEIGHT = 12.0;
    public static double GRAVITY = 386.1;

    public static double k = 0.4;

    final double GOAL_HEIGHT = 38.75;          // inches
    final double ANGLE_BIAS_DEG = -18.35;       // a
    final double G_INCHES = 386.088;            // gravity
    final double VECTOR_TUNE = 2.49;

    public static double GOAL_X = 15;

    public static double GOAL_Y = 135;

    public static boolean sixSeven = false;


    // ===== Dashboard Tunables =====
    public static double targetTicksPerSec = 1000.0;
    public static double desiredHoodAngle = 30.0;
    public static boolean enableShooter = false; // Safety toggle

    private Follower follower;

    private Sensors sensors;
    private Shooter shooter;
    private Intake intake;
    private List<LynxModule> allHubs;

    // Debounce state
    private boolean lastButton = false;

    @Override
    public void init() {
        // 1. Setup Hubs for Manual Caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Initialize Subsystems
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        sensors.update();

        // Shooter constructor now handles PID initialization correctly
        shooter = new Shooter(hardwareMap, telemetry);

        shooter.update(sensors.getFlywheelVelo(), shooter.getCurrentTurretPosition());

        intake = new Intake(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());

        telemetry.addLine("Initialized.");
        telemetry.addLine("Hood will move immediately on START.");
        telemetry.addLine("Press Square/X to toggle Flywheel.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- STEP 1: REFRESH HARDWARE DATA ---
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        sensors.update();

        follower.update();

        // --- STEP 2: HANDLE INPUTS ---

        // Debounced Toggle for Square (PS) or X (Xbox)
        boolean currentButton = gamepad1.square || gamepad1.x;
        if (currentButton && !lastButton) {
            enableShooter = !enableShooter;
        }
        lastButton = currentButton;

        // Tuning Controls
        if (gamepad1.dpad_up) targetTicksPerSec += 5000;
        if (gamepad1.dpad_down) targetTicksPerSec -= 5000;
        if (gamepad1.right_bumper) desiredHoodAngle += 2;
        if (gamepad1.left_bumper) desiredHoodAngle -= 2;
        if (gamepad1.circle){
            targetTicksPerSec +=1000;
        }
        if (gamepad1.triangle){
            targetTicksPerSec -=1000;
        }

        if (gamepad1.right_trigger > 0.2){
            intake.doTestShooter();
        }
        else{
            intake.doIntakeHalt();
        }




        // --- STEP 3: APPLY LOGIC TO SUBSYSTEM ---
        double currentVelo = sensors.getFlywheelVelo();
        double currentTurret = shooter.getCurrentTurretPosition();

        if (gamepad1.circle) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

        // Always update hood angle so it responds even if flywheel is off
        shooter.setHoodTargetAngle(desiredHoodAngle);

        if (enableShooter) {
            if (sixSeven){
                shooter.setTargetVelocityTicks(flywheelTicksFromDistance(follower.getPose().getX(),follower.getPose().getY(), GOAL_X,GOAL_Y));
            }
            else {
                shooter.setTargetVelocityTicks(targetTicksPerSec);
            }
        } else {
            shooter.stopFlywheel();
        }

        // Write to hardware (Servos + Motors)
        shooter.update(currentVelo, currentTurret);

        // --- STEP 4: PHYSICS CALCULATIONS (K-FACTOR) ---
        double deltaY = GOAL_HEIGHT - LAUNCH_HEIGHT;
        double angleRad = Math.toRadians(desiredHoodAngle);
        double cosA = Math.cos(angleRad);

        // Denominator: 2 * cos²(θ) * (x * tan(θ) - y)
        double denom = 2 * Math.pow(cosA, 2) * (TEST_DISTANCE_INCHES * Math.tan(angleRad) - deltaY);

        telemetry.addData("STATUS", enableShooter ? "RUNNING" : "STOPPED (Press Square/X)");

        if (denom > 0.0001) {
            // Basic kinematic equation for projectile motion
            double vReqMuzzle = Math.sqrt((GRAVITY * Math.pow(TEST_DISTANCE_INCHES, 2)) / denom);

            // k = muzzleVelocity / flywheelVelocity
            double calculatedK = (currentVelo > 50) ? (vReqMuzzle / currentVelo) : 0;

            telemetry.addLine("--- PHYSICS ---");
            telemetry.addData("Req Muzzle Speed", "%.2f in/s", vReqMuzzle);
            telemetry.addData("CALCULATED K", "%.8f", calculatedK);
        } else {
            telemetry.addLine("GEOMETRY ERROR: Angle too low for distance!");
        }

        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );


        telemetry.addLine(followerData);
        telemetry.addData("shottime",calculateShotTime(targetTicksPerSec,Math.toRadians(desiredHoodAngle),follower.getPose().getX(),follower.getPose().getY(),k));
        telemetry.addLine("--- HARDWARE ---");
        telemetry.addData("Target Ticks", "%.1f", targetTicksPerSec);
        telemetry.addData("Actual Ticks", "%.1f", currentVelo);
        telemetry.addData("Target Angle", "%.1f", desiredHoodAngle);
        telemetry.addData("Turret Pos", "%.1f", currentTurret);
        telemetry.addData("FLywheel ticks from distance skethy thing", flywheelTicksFromDistance(follower.getPose().getX(),follower.getPose().getY(), GOAL_X,GOAL_Y));


        telemetry.update();
    }


    public  double flywheelTicksFromDistance(double robotX, double robotY, double goalX, double goalY) {
        double distanceInches = Math.hypot(goalX - robotX, goalY - robotY);
        // --- constants (copied from your OpMode) ---
        // tune factor

        // Prevent nonsense
        distanceInches = Math.max(distanceInches, 1.0);

        // Initial physics angle
        double angle = Math.atan(
                2 * GOAL_HEIGHT / distanceInches
                        - Math.tan(Math.toRadians(ANGLE_BIAS_DEG))
        );

        // Clamp to hood limits
        angle = MathFunctions.clamp(
                angle,
                Math.toRadians(10),
                Math.toRadians(85)
        );

        double cos = Math.cos(angle);
        double denom = 2 * cos * cos *
                (distanceInches * Math.tan(angle) - GOAL_HEIGHT);

        if (denom <= 0) return 0;

        // Muzzle velocity (in/s)
        double velocity = Math.sqrt(
                G_INCHES * distanceInches * distanceInches / denom
        );

        // Convert to ticks
        return shooter.calcFlywheelSpeedTicks(
                velocity * VECTOR_TUNE
        );
    }

    static final double TICKS_PER_REV = 4096.0;
    static final double WHEEL_RADIUS_IN = 1.4173; // 72mm / 2 -> inches
    static final double G = 386.0; // in/s^2

    public static double calculateShotTime(
            double ticksPerSec,
            double hoodAngleRad,
            double robotX,double robotY,
            double k
    ) {

        double distanceInches = Math.hypot(GOAL_X - robotX, GOAL_Y - robotY);


        double v0 =
                k *
                        (2.0 * Math.PI * WHEEL_RADIUS_IN * ticksPerSec)
                        / TICKS_PER_REV;

        double vx = v0 * Math.cos(hoodAngleRad);
        return distanceInches / vx;
    }
}