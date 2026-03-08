package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Use this OpMode to find the raw servo positions (0.0–1.0) that correspond to
 * the physical -180° and +180° turret limits. Once found, copy those values into
 * turretServoConfigure as turret1Min/Max and turret2Min/Max.
 *
 * WORKFLOW:
 *  1. Enable "Lock Together" (lockTogether = true) to move both servos at once,
 *     or disable it to tune each independently.
 *  2. Adjust rawPosition (or rawPosition1 / rawPosition2) from the dashboard.
 *  3. Physically drive the turret to the -180° hard stop → note the raw position → that is your Min.
 *  4. Drive to the +180° hard stop → note the raw position → that is your Max.
 *  5. Enter your found min/max below as hypotheticalMin/Max to verify the angle mapping looks correct.
 */
@TeleOp(name = "Turret Servo Range Finder", group = "Test")
@Configurable
public class TurretServoRangeFinder extends OpMode {

    // --- Dashboard Controls ---

    // When true, both servos track rawPosition together
    public static boolean lockTogether = true;
    public static double rawPosition  = 0.5; // used when lockTogether = true
    public static double rawPosition1 = 0.5; // turret1 only, used when lockTogether = false
    public static double rawPosition2 = 0.5; // turret2 only, used when lockTogether = false

    // Enter your candidate min/max here to verify the angle mapping
    public static double hypotheticalMin1 = 0.0;
    public static double hypotheticalMax1 = 1.0;
    public static double hypotheticalMin2 = 0.0;
    public static double hypotheticalMax2 = 1.0;
    public static boolean powerToggle = true;

    // ---

    private Servo turret1;
    private Servo turret2;

    @Override
    public void init() {
        turret1 = hardwareMap.get(Servo.class, "turretServo1");
        turret2 = hardwareMap.get(Servo.class, "turretServo2");
        turret2.setDirection(Servo.Direction.REVERSE);


        telemetry.addLine("Ready — adjust rawPosition on dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
        double pos1 = lockTogether ? rawPosition : rawPosition1;
        double pos2 = lockTogether ? rawPosition : rawPosition2;

        pos1 = Range.clip(pos1, 0.0, 1.0);
        pos2 = Range.clip(pos2, 0.0, 1.0);

        if (powerToggle) {
            turret1.setPosition(pos1);
            turret2.setPosition(pos2);
        }


        // Compute angle each servo maps to given the hypothetical min/max
        double angle1 = scale(pos1, hypotheticalMin1, hypotheticalMax1, -180, 180);
        double angle2 = scale(pos2, hypotheticalMin2, hypotheticalMax2, -180, 180);
        double averageAngle = (angle1 + angle2) / 2.0;

        telemetry.addLine("=== Raw Servo Positions ===");
        telemetry.addData("Turret 1 Raw", String.format("%.4f", pos1));
        telemetry.addData("Turret 2 Raw", String.format("%.4f", pos2));
        telemetry.addLine("");
        telemetry.addLine("=== Angle Preview (using hypothetical min/max) ===");
        telemetry.addData("Turret 1 Angle (deg)", String.format("%.1f", angle1));
        telemetry.addData("Turret 2 Angle (deg)", String.format("%.1f", angle2));
        telemetry.addData("Average Angle  (deg)", String.format("%.1f", averageAngle));
        telemetry.addLine("");
        telemetry.addLine("=== Hypothetical Min/Max ===");
        telemetry.addData("Turret 1 Min / Max", String.format("%.4f  /  %.4f", hypotheticalMin1, hypotheticalMax1));
        telemetry.addData("Turret 2 Min / Max", String.format("%.4f  /  %.4f", hypotheticalMin2, hypotheticalMax2));
        telemetry.update();
    }

    // Range.scale equivalent without clamping, so out-of-range values are still visible
    private double scale(double value, double inMin, double inMax, double outMin, double outMax) {
        if (inMin == inMax) return outMin;
        return outMin + (value - inMin) / (inMax - inMin) * (outMax - outMin);
    }
}
