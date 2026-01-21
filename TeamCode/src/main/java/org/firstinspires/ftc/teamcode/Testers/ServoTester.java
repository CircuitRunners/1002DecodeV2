package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Test")
public class ServoTester extends OpMode {

    private GamepadEx player1;
    private DcMotorEx intake;
    private Servo servo;

    private double servoPosition = 0;


    @Override
    public void init(){
        player1 = new GamepadEx(gamepad1);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "gateRight");
        //servo = hardwareMap.get(Servo.class, "hoodServo");

        servo.setPosition(0);
        servo.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();

        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            servoPosition -= 0.01;
            servoPosition = Range.clip(servoPosition,0,1);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            servoPosition += 0.01;
            servoPosition = Range.clip(servoPosition,0,1);
        }

        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)){
            servoPosition = 0;
            servoPosition = Range.clip(servoPosition,0,1);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)){
            servoPosition = 1;
            servoPosition = Range.clip(servoPosition,0,1);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.CROSS)){
            servoPosition = 0.5;
            servoPosition = Range.clip(servoPosition,0,1);
        }

        servo.setPosition(servoPosition);

        telemetry.addLine("Left bumper to decrease, Right Bumper to increase, triangle for 0.0, square for 1.0, cross for 0.5");

        telemetry.addData("Servo Pos", servo.getPosition());


        telemetry.update();
    }

}
