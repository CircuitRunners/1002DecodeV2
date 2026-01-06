package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

@TeleOp(name="Shooter: K-Tuner (4096 Ticks)", group="Calibration")
public class ShooterKTuner extends LinearOpMode {

    // ADJUST THESE TO YOUR TEST SETUP
    private static final double TEST_DISTANCE_INCHES = 120.0; // 10 Feet
    private static final double GOAL_HEIGHT = 38.75; 
    private static final double LAUNCH_HEIGHT = 12.0;
    private static final double GRAVITY = 386.1;

    private Shooter shooter;
    private double testTicksPerSec = 60000; // Starting point (~880 RPM)
    private double testAngle = 30.0;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap, telemetry);
        
        telemetry.addLine("Controls:");
        telemetry.addLine("DPAD Up/Down: +/- 1000 Ticks/Sec");
        telemetry.addLine("Bumpers: +/- 1 Degree Hood");
        telemetry.addLine("X Button: STOP Flywheel");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Adjust Velocity (Large steps for 4096 encoder)
            if (gamepad1.dpad_up) testTicksPerSec += 500;
            if (gamepad1.dpad_down) testTicksPerSec -= 500;
            
            // 2. Adjust Angle
            if (gamepad1.right_bumper) testAngle += 0.5;
            if (gamepad1.left_bumper) testAngle -= 0.5;
            
            // 3. Safety Stop
            if (gamepad1.x) testTicksPerSec = 0;

            // 4. Update Subsystem
            shooter.setTargetVelocityTicks(testTicksPerSec);
            shooter.setHoodTargetAngle(testAngle);
            
            // Assuming you add a way to get current encoder velocity in your Shooter class
            // If not, use: shooter1.getVelocity()
            shooter.update(testTicksPerSec, 180); // Passing target as current for simplicity

            // 5. THE PHYSICS CALCULATION
            // We find the THEORETICAL muzzle velocity (in/s) needed for this distance/angle
            double deltaY = GOAL_HEIGHT - LAUNCH_HEIGHT;
            double angleRad = Math.toRadians(testAngle);
            double cosA = Math.cos(angleRad);
            
            double denominator = 2 * cosA * cosA * (TEST_DISTANCE_INCHES * Math.tan(angleRad) - deltaY);
            
            if (denominator > 0) {
                double vReqMuzzleInchesPerSec = Math.sqrt((GRAVITY * TEST_DISTANCE_INCHES * TEST_DISTANCE_INCHES) / denominator);
                
                // k = V_muzzle / Flywheel_Ticks_Per_Sec
                double calculatedK = vReqMuzzleInchesPerSec / testTicksPerSec;

                telemetry.addData("STATUS", testTicksPerSec > 0 ? "FIRING" : "STOPPED");
                telemetry.addData("Target Ticks/Sec", "%.0f", testTicksPerSec);
                telemetry.addData("Hood Angle", "%.1f deg", testAngle);
                telemetry.addLine("--------------------------------");
                telemetry.addLine("INSTRUCTIONS:");
                telemetry.addLine("1. Place robot exactly 10ft from goal.");
                telemetry.addLine("2. Adjust speed/angle until you hit center.");
                telemetry.addData("3. COPY THIS K TO TABLE", "%.8f", calculatedK);
            } else {
                telemetry.addLine("ANGLE TOO LOW TO REACH GOAL");
            }
            
            telemetry.update();
        }
    }
}