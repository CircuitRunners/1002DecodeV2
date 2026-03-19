package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@TeleOp(name = "IntakeServoFinder", group = "TEST")
public class IntakeServoFinder extends OpMode {

    private GamepadEx player1;

    private Servo directionSwitch;
    private Servo centerClutch;
    private Servo gateLeft;
    private Servo gateRight;

    // Current values from Intake.java
    private double dirSwitchPos  = 0.35; // TRANSFER_DIRECTION_TRANSFER_POS
    private double clutchPos     = 0.59; // TRANSFER_ON
    private double gateLeftPos   = 0.81; // GATE_CLOSED
    private double gateRightPos  = 0.81; // GATE_CLOSED

    private enum ActiveServo { DIRECTION_SWITCH, CENTER_CLUTCH, GATE_LEFT, GATE_RIGHT }
    private ActiveServo active = ActiveServo.DIRECTION_SWITCH;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);

        directionSwitch = hardwareMap.get(Servo.class, "directionSwitch");
        directionSwitch.setDirection(Servo.Direction.REVERSE);

        centerClutch = hardwareMap.get(Servo.class, "centerClutch");
        centerClutch.setDirection(Servo.Direction.REVERSE);

        gateLeft = hardwareMap.get(Servo.class, "gateLeft");
        gateLeft.setDirection(Servo.Direction.REVERSE);

        gateRight = hardwareMap.get(Servo.class, "gateRight");
        gateRight.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("IntakeServoFinder ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();

        // --- Select active servo (D-pad left/right) ---
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            active = ActiveServo.values()[(active.ordinal() + 1) % ActiveServo.values().length];
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            active = ActiveServo.values()[(active.ordinal() + ActiveServo.values().length - 1) % ActiveServo.values().length];
        }

        // --- Adjust position ---
        double delta = 0;
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) delta = +0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))  delta = -0.01;

        // --- Snap presets for each servo ---
        switch (active) {
            case DIRECTION_SWITCH:
                dirSwitchPos = Range.clip(dirSwitchPos + delta, 0, 1);
                if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) dirSwitchPos = 0.35; // TRANSFER
                if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))   dirSwitchPos = 0.43; // CYCLE
                if (player1.wasJustPressed(GamepadKeys.Button.CROSS))    dirSwitchPos = 0.5;
                directionSwitch.setPosition(dirSwitchPos);
                break;

            case CENTER_CLUTCH:
                clutchPos = Range.clip(clutchPos + delta, 0, 1);
                if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) clutchPos = 0.59; // TRANSFER_ON
                if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))   clutchPos = 0.67; // TRANSFER_OFF
                if (player1.wasJustPressed(GamepadKeys.Button.CROSS))    clutchPos = 0.5;
                centerClutch.setPosition(clutchPos);
                break;

            case GATE_LEFT:
                gateLeftPos = Range.clip(gateLeftPos + delta, 0, 1);
                if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) gateLeftPos = 0.81; // CLOSED
                if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))   gateLeftPos = 0.15; // OPEN
                if (player1.wasJustPressed(GamepadKeys.Button.CROSS))    gateLeftPos = 0.5;
                gateLeft.setPosition(gateLeftPos);
                break;

            case GATE_RIGHT:
                gateRightPos = Range.clip(gateRightPos + delta, 0, 1);
                if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) gateRightPos = 0.81; // CLOSED
                if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))   gateRightPos = 0.15; // OPEN
                if (player1.wasJustPressed(GamepadKeys.Button.CROSS))    gateRightPos = 0.5;
                gateRight.setPosition(gateRightPos);
                break;
        }

        // --- Telemetry ---
        telemetry.addLine("D-pad L/R = switch servo  |  LB/RB = -/+0.01");
        telemetry.addLine("Triangle = preset A  |  Square = preset B  |  Cross = 0.5");
        telemetry.addLine("");
        telemetry.addData("ACTIVE", active);
        telemetry.addLine("");

        telemetry.addData((active == ActiveServo.DIRECTION_SWITCH ? ">>> " : "    ") + "directionSwitch", "%.3f  [T:0.35 | C:0.43]", dirSwitchPos);
        telemetry.addData((active == ActiveServo.CENTER_CLUTCH    ? ">>> " : "    ") + "centerClutch",    "%.3f  [ON:0.59 | OFF:0.67]", clutchPos);
        telemetry.addData((active == ActiveServo.GATE_LEFT        ? ">>> " : "    ") + "gateLeft",        "%.3f  [CLOSED:0.81 | OPEN:0.15]", gateLeftPos);
        telemetry.addData((active == ActiveServo.GATE_RIGHT       ? ">>> " : "    ") + "gateRight",       "%.3f  [CLOSED:0.81 | OPEN:0.15]", gateRightPos);

        telemetry.update();
    }
}
