package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter: K-Tuner v2.1", group = "TEST")
public class ShooterKTuner extends OpMode {

    // ===== Physics Constants (Standardized for 10ft Test) =====
    public static double TEST_DISTANCE_INCHES = 120.0;
    public static double GOAL_HEIGHT = 38.75;
    public static double LAUNCH_HEIGHT = 12.0;
    public static double GRAVITY = 386.1;

    // ===== Dashboard Tunables =====
    public static double targetTicksPerSec = 1000.0;
    public static double desiredHoodAngle = 30.0;
    public static boolean enableShooter = false; // Safety toggle

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

        shooter.update(shooter.getFlywheelVelo(), shooter.getCurrentTurretPosition());

        intake = new Intake(hardwareMap, telemetry);

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
        double currentVelo = shooter.getFlywheelVelo();
        double currentTurret = shooter.getCurrentTurretPosition();

        // Always update hood angle so it responds even if flywheel is off
        shooter.setHoodTargetAngle(desiredHoodAngle);

        if (enableShooter) {
            shooter.setTargetVelocityTicks(targetTicksPerSec);
        } else {
            // This sets the internal target to 0 and cuts power
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

        telemetry.addLine("--- HARDWARE ---");
        telemetry.addData("Target Ticks", "%.1f", targetTicksPerSec);
        telemetry.addData("Actual Ticks", "%.1f", currentVelo);
        telemetry.addData("Target Angle", "%.1f", desiredHoodAngle);
        telemetry.addData("Turret Pos", "%.1f", currentTurret);
        telemetry.update();
    }
}