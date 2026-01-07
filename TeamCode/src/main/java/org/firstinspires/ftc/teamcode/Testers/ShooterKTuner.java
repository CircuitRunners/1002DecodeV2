package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter: K-Tuner ", group = "TEST")
public class ShooterKTuner extends OpMode {

    // ===== Physics Constants (Standardized for 10ft Test) =====
    public static double TEST_DISTANCE_INCHES = 120.0;
    public static double GOAL_HEIGHT = 38.75;
    public static double LAUNCH_HEIGHT = 12.0;
    public static double GRAVITY = 386.1;

    // ===== Dashboard Tunables (Start high for 4096 encoder) =====
    public static double targetTicksPerSec = 200000;
    public static double targetHoodAngle = 30.0;

    private Sensors sensors = new Sensors();
    private Shooter shooter;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        shooter = new Shooter(hardwareMap, telemetry);
        sensors.init(hardwareMap, "SRSHub");

        telemetry.addLine("Ready. Place robot exactly 10ft (120in) from goal. fix turret at 0 degrees (forward over intake)");
        telemetry.addLine("Gamepad 1: DPAD Up/Down (Speed), Bumpers (Angle)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear bulk cache for highest frequency updates
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        sensors.update();

        // 1. GAMEPAD TUNING (Adjust targets live)
        if (gamepad1.dpad_up) targetTicksPerSec += 200;
        if (gamepad1.dpad_down) targetTicksPerSec -= 200;
        if (gamepad1.right_bumper) targetHoodAngle += 0.5;
        if (gamepad1.left_bumper) targetHoodAngle -= 0.5;

        // Clip the values to safe hardware limits
        targetTicksPerSec = Range.clip(targetTicksPerSec, 0, 385000);
        targetHoodAngle = Range.clip(targetHoodAngle, 0, 45);

        // 2. APPLY TO HARDWARE
        shooter.setTargetVelocityTicks(targetTicksPerSec);
        shooter.setHoodTargetAngle(targetHoodAngle);

        // Snapshot the actual measured speed for the k-calculation
        double currentTicksPerSec = sensors.getFlywheelVelo();
        shooter.update(currentTicksPerSec, 0); // Turret fixed at 180 for tuning

        // 3. THE PHYSICS CHARACTERIZATION
        double deltaY = GOAL_HEIGHT - LAUNCH_HEIGHT;
        double angleRad = Math.toRadians(targetHoodAngle);
        double cosA = Math.cos(angleRad);

        // Denominator: 2 * cos²(θ) * (x * tan(θ) - y)
        double denominator = 2 * Math.pow(cosA, 2) * (TEST_DISTANCE_INCHES * Math.tan(angleRad) - deltaY);

        if (denominator > 0.0001) {
            // v = sqrt( (g * x²) / Denominator )
            double vReqMuzzleInchesPerSec = Math.sqrt((GRAVITY * Math.pow(TEST_DISTANCE_INCHES, 2)) / denominator);

            // k = (Theoretical in/s) / (Measured ticks/s)
            // This is the constant that "bridges" your specific shooter to pure physics.
            double calculatedK = 0;
            if (currentTicksPerSec > 100) { // Avoid division by zero when stopped
                calculatedK = vReqMuzzleInchesPerSec / currentTicksPerSec;
            }

            telemetry.addData("--- TARGETS ---", "");
            telemetry.addData("Target Ticks/s", "%.0f", targetTicksPerSec);
            telemetry.addData("Target Angle", "%.2f deg", targetHoodAngle);

            telemetry.addData("--- MEASURED ---", "");
            telemetry.addData("Measured Ticks/s", "%.2f", currentTicksPerSec);
            telemetry.addData("Physics Req Speed", "%.2f in/s", vReqMuzzleInchesPerSec);

            telemetry.addLine("--------------------------------");
            telemetry.addData("CALCULATED K", "%.10f", calculatedK);
            telemetry.addLine("--------------------------------");
            telemetry.addLine("DIRECTIONS:");
            telemetry.addLine("1. Dial speed/angle until you hit dead-center.");
            telemetry.addLine("2. Note the 'K' and put it in MUZZLE_K_TABLE with the associated angle");
        } else {
            telemetry.addLine("ERROR: Geometry impossible. Increase Angle.");
        }

        telemetry.update();
    }
}