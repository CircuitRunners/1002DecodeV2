package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;

@TeleOp(name = "IndexerTester", group = "TEST")
public class IndexerTester extends OpMode {

    private GamepadEx player1;
    private Intake intake;
    private Sensors sensors;

    private LimelightCamera.BallOrder ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap, telemetry);
//        sensors = new Sensors();
//        sensors.init(hardwareMap, "SRSHub");

        intake.setCanShoot(false);

        telemetry.addLine("IndexerTester ready.");
        telemetry.addLine("RIGHT TRIGGER  = Transfer (indexer forward)");
        telemetry.addLine("LEFT TRIGGER   = Outtake");
        telemetry.addLine("CROSS          = Intake");
        telemetry.addLine("CIRCLE         = Cycle");
        telemetry.addLine("LEFT BUMPER    = canShoot ON");
        telemetry.addLine("RIGHT BUMPER   = canShoot OFF");
        telemetry.addLine("SQUARE         = Reset state");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();
//        sensors.update();
//
//        DetectedColor s1 = sensors.getDetectedColor(sensors.colorSensor1);
//        DetectedColor s2 = sensors.getDetectedColor(sensors.colorSensor2);
//        DetectedColor s3 = sensors.getDetectedColor(sensors.colorSensor3);

        // --- Controls ---
        if (gamepad1.right_trigger > 0.2) {
            intake.doIntake(); // runs transfer (indexer forward to shooter)
        } else if (gamepad1.left_trigger > 0.2) {
            intake.doOuttake();
        } else if (player1.isDown(GamepadKeys.Button.CROSS)) {
            intake.doTestShooter();
        } else if (player1.isDown(GamepadKeys.Button.CIRCLE)) {
            intake.doCycle();
        } else if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            intake.resetState();
        }

        if (gamepad1.left_bumper) {
            intake.setCanShoot(true);
        } else if (gamepad1.right_bumper) {
            intake.setCanShoot(false);
        }

        intake.update(true, ballOrder, DetectedColor.NONE, DetectedColor.NONE, DetectedColor.NONE);

        // --- Telemetry ---
        telemetry.addLine("=== INDEXER TESTER ===");
        telemetry.addData("Intake State", intake.getCurrentIntakeState());
        telemetry.addData("canShoot", Intake.canShoot);
        telemetry.addLine("");
        telemetry.addLine("--- Color Sensors ---");
//        telemetry.addData("Slot 1", s1 != null ? s1 : "NONE");
//        telemetry.addData("Slot 2", s2 != null ? s2 : "NONE");
//        telemetry.addData("Slot 3", s3 != null ? s3 : "NONE");
//        telemetry.addData("Proximity 1", sensors.getColor1Proximity());
//        telemetry.addData("Proximity 2", sensors.getColor2Proximity());
//        telemetry.addData("Proximity 3", sensors.getColor3Proximity());
        telemetry.addLine("");
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("R.Trigger=Transfer  L.Trigger=Outtake");
        telemetry.addLine("Cross=Intake  Circle=Cycle  Square=Reset");
        telemetry.addLine("L.Bumper=canShoot ON  R.Bumper=canShoot OFF");
        telemetry.update();
    }
}
