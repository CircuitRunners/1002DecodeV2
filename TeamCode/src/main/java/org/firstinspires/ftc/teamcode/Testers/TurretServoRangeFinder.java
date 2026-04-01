package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

/**
 * Simple servo position finder with left/right gamepad control.
 * Use LEFT_STICK_X to move servo, displays raw position and mapped angle.
 */
@TeleOp(name = "Turret Servo Range Finder", group = "Test")
@Configurable
public class TurretServoRangeFinder extends OpMode {

    // --- Dashboard Controls ---
    public  static double servoPosition = 0.5; // current servo position (0.0-1.0)
    public static double minPosition = 0.0;   // min raw position maps to -180°
    public static double maxPosition = 0.98;   // max raw position maps to +180°
    public GamepadEx player1;

    // ---

    private Servo turret1;
    private Servo turret2;

    @Override
    public void init() {
        turret1 = hardwareMap.get(Servo.class, "turretLeft");
        turret1.setDirection(Servo.Direction.REVERSE);
        turret2 = hardwareMap.get(Servo.class, "turretRight");
        turret2.setDirection(Servo.Direction.REVERSE);
        player1 = new GamepadEx(gamepad1);
        telemetry.addLine("Ready — use LEFT/RIGHT to adjust position");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();
        
        // Left/Right stick to adjust position
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            servoPosition -= 0.005;
        } else if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            servoPosition += 0.005;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            servoPosition += 0.01;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            servoPosition -= 0.01;
        }
        
        // Clamp position to valid servo range
        servoPosition = Range.clip(servoPosition, 0.0, 1.0);
        
        // Set servo position
        turret1.setPosition(servoPosition);
        turret2.setPosition(servoPosition);
        
        // Map position to angle using min/max
        double angle = scale(servoPosition, minPosition, maxPosition, -178, 166);
        
        // Display
        telemetry.addLine("=== Turret Servo Position ===");
        telemetry.addData("Raw Position", String.format("%.4f", servoPosition));
        telemetry.addData("Mapped Angle (deg)", String.format("%.1f", angle));
        telemetry.addLine("");
        telemetry.addLine("=== Calibration Values ===");
        telemetry.addData("Min / Max Position", String.format("%.4f  /  %.4f", minPosition, maxPosition));
        telemetry.addLine("");
        telemetry.addLine("Use DPAD LEFT/RIGHT to adjust");
        telemetry.update();
    }

    // Range.scale equivalent without clamping, so out-of-range values are still visible
    private double scale(double value, double inMin, double inMax, double outMin, double outMax) {
        if (inMin == inMax) return outMin;
        return outMin + (value - inMin) / (inMax - inMin) * (outMax - outMin);
    }
}
