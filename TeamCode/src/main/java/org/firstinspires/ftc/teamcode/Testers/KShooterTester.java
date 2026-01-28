package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

import java.util.List;
@Disabled
@Configurable
@TeleOp(name = "Shooter: K-Tuner but sigma", group = "TEST")
public class KShooterTester extends OpMode {

        // ===== Physics Constants =====
        public static double TEST_DISTANCE_INCHES = 120.0;
        public static double GOAL_HEIGHT = 38.75;
        public static double LAUNCH_HEIGHT = 11.6;
        public static double GRAVITY = 386.088;

        // ===== Dashboard Tunables =====
        public static double targetTicksPerSec = 250000.0; // Starting at your typical range
        public static double desiredHoodAngle = 30.0;
        public static boolean enableShooter = false;

        private Sensors sensors;
        private Shooter shooter;
        private Intake intake;
        private List<LynxModule> allHubs;
        private ElapsedTime runtime = new ElapsedTime();

        private boolean lastButton = false;
        private boolean lastLogButton = false;

        @Override
        public void init() {
            allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            sensors = new Sensors();
            sensors.init(hardwareMap, "SRSHub");

            shooter = new Shooter(hardwareMap, telemetry);
            intake = new Intake(hardwareMap, telemetry);

            telemetry.setMsTransmissionInterval(50); // Faster telemetry for tuning
            telemetry.addLine("Ready to Tune.");
            telemetry.update();
        }

        @Override
        public void loop() {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            sensors.update();

            // --- 1. SMART CONTROLS ---

            // Toggle Flywheel (Square/X)
            if (gamepad1.square || gamepad1.x && !lastButton) enableShooter = !enableShooter;
            lastButton = gamepad1.square || gamepad1.x;

            // FAST Adjust (D-Pad): 10k increments for high-speed motors
            if (gamepad1.dpad_up) targetTicksPerSec += 10000;
            if (gamepad1.dpad_down) targetTicksPerSec -= 10000;

            // FINE Adjust (Triangle/Circle): 1k increments
            if (gamepad1.triangle) targetTicksPerSec += 1000;
            if (gamepad1.circle) targetTicksPerSec -= 1000;

            // Hood Adjust (Bumpers)
            if (gamepad1.right_bumper) desiredHoodAngle += 0.5;
            if (gamepad1.left_bumper) desiredHoodAngle -= 0.5;

            // --- 2. SUBSYSTEM UPDATE ---
            double currentVelo = sensors.getFlywheelVelo();
            double currentTurret = shooter.getCurrentTurretPosition();

            shooter.setHoodTargetAngle(desiredHoodAngle);
            if (enableShooter) {
                shooter.setTargetVelocityTicks(targetTicksPerSec);
            } else {
                shooter.stopFlywheel();
            }
            shooter.update(currentVelo, currentTurret);

            // --- 3. PHYSICS & K-FACTOR CALCULATION ---
            double y = GOAL_HEIGHT - LAUNCH_HEIGHT;
            double theta = Math.toRadians(desiredHoodAngle);

            // V_req = sqrt( (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y)) )
            double cosSquared = Math.pow(Math.cos(theta), 2);
            double denom = 2 * cosSquared * (TEST_DISTANCE_INCHES * Math.tan(theta) - y);

            double vReqMuzzle = 0;
            double calculatedK = 0;

            if (denom > 0) {
                vReqMuzzle = Math.sqrt((GRAVITY * Math.pow(TEST_DISTANCE_INCHES, 2)) / denom);
                // We use targetTicksPerSec for the K calculation because currentVelo might jitter,
                // and we want the K relative to our setpoint once it is reached.
                calculatedK = vReqMuzzle / targetTicksPerSec;
            }

            // --- 4. DATA STABILITY INDICATOR ---
            // Helpful to know if the flywheel has settled before reading the K-value
            double error = Math.abs(targetTicksPerSec - currentVelo);
            boolean isStable = error < (targetTicksPerSec * 0.05); // 2% tolerance

            // --- 5. TELEMETRY PRO ---
            telemetry.addLine("== SYSTEM STATUS ==");
            telemetry.addData("Shooter State", enableShooter ? "ACTIVE" : "IDLE");
            telemetry.addData("Flywheel Stability", isStable ? "READY TO LOG" : "STABILIZING...");

            telemetry.addLine("\n== TUNING VALUES ==");
            telemetry.addData("Target Ticks/Sec", "%.0f", targetTicksPerSec);
            telemetry.addData("Current Ticks/Sec", "%.0f", currentVelo);
            telemetry.addData("Hood Angle", "%.2f°", desiredHoodAngle);
            telemetry.addData("shottime",calculateShotTime(targetTicksPerSec,Math.toRadians(desiredHoodAngle),16,38.75,1));

            telemetry.addLine("\n== CALCULATED PHYSICS ==");
            if (denom <= 0) {
                telemetry.addLine("IMPOSSIBLE GEOMETRY: Increase angle or decrease distance.");
            } else {
                telemetry.addData("Req. Muzzle Vel", "%.2f in/s", vReqMuzzle);
                telemetry.addData("RESULTING K-FACTOR", "%.10f", calculatedK);
            }

            // Visual cue for the table format
            telemetry.addLine("\n== TABLE ENTRY (Angle, K) ==");
            telemetry.addLine(String.format("{%.1f, %.8f}", desiredHoodAngle, calculatedK));

            telemetry.update();

            // --- 6. INTAKE / TRANSFER CONTROL ---
            if (gamepad1.right_trigger > 0.2) intake.doTestShooter();
            else intake.doIntakeHalt();
        }


    static final double TICKS_PER_REV = 4096.0;
    static final double WHEEL_RADIUS_IN = 1.4173; // 72mm / 2 -> inches
    static final double G = 386.0; // in/s^2
    public static double calculateShotTime(
            double ticksPerSec,
            double hoodAngleRad,
            double launchHeightIn,
            double goalHeightIn,
            double k
    ) {
        // 1) ticks/sec -> launch velocity (in/s)
        double v0 =
                k *
                        (2.0 * Math.PI * WHEEL_RADIUS_IN * ticksPerSec)
                        / TICKS_PER_REV;
        // 2) Quadratic coefficients
        double a = 0.5 * G;
        double b = -v0 * Math.sin(hoodAngleRad);
        double c = goalHeightIn - launchHeightIn;
        // 3) Discriminant check
        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0) {
            return Double.NaN; // shot cannot reach goal
        }
        // 4) Take the LARGER root (ball hits on way down)
        double sqrtD = Math.sqrt(discriminant);
        double t1 = (-b + sqrtD) / (2.0 * a);
        double t2 = (-b - sqrtD) / (2.0 * a);
        return Math.max(t1, t2);
    }
    }
