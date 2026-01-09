package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;

@TeleOp( group = "TEST")
public class SigmaIntakeTest extends OpMode {

    private GamepadEx player1;
    private Intake intake;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        player1.readButtons();

        // Hold Cross (A) to Intake
        if (player1.isDown(GamepadKeys.Button.CROSS)) {
            intake.intake();
            telemetry.addData("Status", "Intaking...");
        }
        // Hold Circle (B) to Cycle
        else if (player1.isDown(GamepadKeys.Button.CIRCLE)) {
            intake.cycle();
            telemetry.addData("Status", "Cycling...");
        }
        // Otherwise, stop
        else {
            intake.intakeMotorIdle();
            telemetry.addData("Status", "Idle");
        }

        telemetry.update();
    }
}