package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

/**
 * PROJECTILE MOTION TUNER (K-TUNER)
 * * CONCEPT:
 * We use physics to calculate the required "Muzzle Velocity" (v) in inches per second.
 * However, we control the motor in "Encoder Ticks per Second."
 * * The variable 'k' is the CHARACTERIZATION CONSTANT. It represents the ratio between
 * the theoretical output and the electrical input, accounting for wheel diameter,
 * gear ratios, and energy loss (friction/slip).
 */
@TeleOp(name="Shooter: K-Tuner (4096 Ticks)", group="Calibration")
public class ShooterKTuner extends LinearOpMode {

    // PHYSICAL CONSTANTS: Measured in inches
    private static final double TEST_DISTANCE_INCHES = 120.0; // Horizontal distance to goal
    private static final double GOAL_HEIGHT = 38.75;          // Target height (Y)
    private static final double LAUNCH_HEIGHT = 12.0;         // Release point height (Y0)
    private static final double GRAVITY = 386.1;              // Acceleration (in/s^2)

    private Shooter shooter;
    private double testTicksPerSec = 60000;
    private double testAngle = 30.0;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap, telemetry);

        telemetry.addLine("Controls:");
        telemetry.addLine("DPAD Up/Down: Adjust Motor Velocity");
        telemetry.addLine("Bumpers: Adjust Hood Launch Angle");
        telemetry.addLine("X Button: EMERGENCY STOP");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- EMPIRICAL TESTING ---
            // Manually find the velocity/angle combination that hits the target
            if (gamepad1.dpad_up) testTicksPerSec += 500;
            if (gamepad1.dpad_down) testTicksPerSec -= 500;

            if (gamepad1.right_bumper) testAngle += 0.5;
            if (gamepad1.left_bumper) testAngle -= 0.5;

            if (gamepad1.x) testTicksPerSec = 0;

            shooter.setTargetVelocityTicks(testTicksPerSec);
            shooter.setHoodTargetAngle(testAngle);
            shooter.update(testTicksPerSec, 180);

            // --- THE PHYSICS MODEL ---
            // Solving for initial velocity (v) in the trajectory equation:
            // y = x*tan(θ) - (g*x²) / (2*v²*cos²(θ))

            double deltaY = GOAL_HEIGHT - LAUNCH_HEIGHT;
            double angleRad = Math.toRadians(testAngle);
            double cosA = Math.cos(angleRad);

            // Calculate the denominator for the velocity formula
            double denominator = 2 * cosA * cosA * (TEST_DISTANCE_INCHES * Math.tan(angleRad) - deltaY);

            if (denominator > 0) {
                // Determine the theoretical Muzzle Velocity needed in inches/second
                double vReqMuzzleInchesPerSec = Math.sqrt((GRAVITY * TEST_DISTANCE_INCHES * TEST_DISTANCE_INCHES) / denominator);

                /*
                 * CALCULATING THE RATIO (k):
                 * k = (Required Linear Velocity) / (Applied Motor Velocity)
                 * * This constant allows us to say:
                 * Target_Ticks = Required_Inches_Per_Sec / k
                 */
                double calculatedK = vReqMuzzleInchesPerSec / testTicksPerSec;

                telemetry.addData("STATUS", testTicksPerSec > 0 ? "STABILIZING" : "IDLE");
                telemetry.addData("Motor Velocity (Ticks/s)", "%.0f", testTicksPerSec);
                telemetry.addData("Launch Angle (Deg)", "%.1f", testAngle);
                telemetry.addLine("--------------------------------");
                telemetry.addLine("TUNING STEPS:");
                telemetry.addLine("1. Verify robot is exactly 10ft away.");
                telemetry.addLine("2. Adjust until the projectile hits the goal center.");
                telemetry.addData("3. USE THIS CHARACTERIZATION CONSTANT", "%.8f", calculatedK);
            } else {
                telemetry.addLine("SYSTEM ERROR: Launch angle is physically incapable of reaching target height.");
            }

            telemetry.update();
        }
    }
}