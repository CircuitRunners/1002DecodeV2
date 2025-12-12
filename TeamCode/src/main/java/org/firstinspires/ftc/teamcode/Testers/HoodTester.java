package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


@TeleOp(name = "HoodTester", group = "TEST")
public class HoodTester extends OpMode {

    private Servo hoodServo;
    private double servoPos;
    private double servoAngle;
    private double targetAngle;
    private double targetPos;
    private static double MIN_LAUNCH_ANGLE_DEG = 15.0;
    private static double MAX_LAUNCH_ANGLE_DEG = 60.0;
    private GamepadEx player1;




    public void init() {

        hoodServo = hardwareMap.get(Servo.class, "hood");
        targetAngle = MIN_LAUNCH_ANGLE_DEG;

        player1 = new GamepadEx(gamepad1);


    }

    public void loop() {
        player1.readButtons();

        servoPos = hoodServo.getPosition();
        servoAngle = Range.scale(servoPos, 0, 1, MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG);

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            targetAngle += 5;
            targetAngle = Range.clip(targetAngle, MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG);
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            targetAngle -= 5;
            targetAngle = Range.clip(targetAngle, MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG);
        }

        targetPos = Range.scale(targetAngle, MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG, 0, 1);

        hoodServo.setPosition(targetPos);

        telemetry.addData("Servo Pos: ", servoPos);
        telemetry.addData("Servo Angle: ", servoAngle);
        telemetry.addData("Target Position: ", targetPos);
        telemetry.addData("Target Angle: ", targetAngle);
        telemetry.update();

    }
}
