package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTester", group = "TEST")
public class MotorTester extends OpMode {
    DcMotor motor;
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    public void loop() {
        motor.setPower(1);
    }


}
