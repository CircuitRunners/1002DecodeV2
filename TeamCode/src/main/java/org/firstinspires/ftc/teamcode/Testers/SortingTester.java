package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;

@TeleOp(name = "SortingTester", group = "TEST")
public class SortingTester extends OpMode {

    private GamepadEx player1;
    private Intake intake;
    private Sensors sensors;
    //private LimelightCamera limelight;

    private LimelightCamera.BallOrder ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;
    private boolean useSensorPattern = false;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
//
//        limelight = new LimelightCamera(hardwareMap);
//        limelight.limelightCamera.start();

        intake = new Intake(hardwareMap, telemetry);

        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        intake.setCanShoot(false);

        telemetry.addLine("SortingTester ready.");
        telemetry.addLine("SQUARE         = Start sort");
        telemetry.addLine("TRIANGLE       = Toggle sensor/manual pattern mode");
        telemetry.addLine("DPAD UP        = BallOrder: GREEN_PURPLE_PURPLE");
        telemetry.addLine("DPAD DOWN      = BallOrder: PURPLE_GREEN_PURPLE");
        telemetry.addLine("DPAD LEFT      = BallOrder: PURPLE_PURPLE_GREEN");
        telemetry.addLine("LEFT BUMPER    = canShoot ON");
        telemetry.addLine("RIGHT BUMPER   = canShoot OFF");
        telemetry.update();
    }

    /** Converts detected colors from the three sensors into a pattern string like "PPG". */
    private String buildPatternFromSensors(DetectedColor b1, DetectedColor b2, DetectedColor b3) {
        return colorToChar(b1) + colorToChar(b2) + colorToChar(b3);
    }

    private String colorToChar(DetectedColor color) {
        if (color == DetectedColor.GREEN) return "G";
        if (color == DetectedColor.PURPLE) return "P";
        else {
            throw new IllegalArgumentException("Invalid color");
        }
    }

    @Override
    public void loop() {
        player1.readButtons();
        sensors.update();

        DetectedColor ball1 = sensors.getDetectedColor(sensors.colorSensor1);
        DetectedColor ball2 = sensors.getDetectedColor(sensors.colorSensor2);
        DetectedColor ball3 = sensors.getDetectedColor(sensors.colorSensor3);

        // Toggle between sensor-derived pattern and hardcoded "PPG"
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            useSensorPattern = !useSensorPattern;
        }

        // One-button sort trigger
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            if (useSensorPattern) {
                boolean sensorsValid = ball1 != DetectedColor.NONE && ball1 != null
                        && ball2 != DetectedColor.NONE && ball2 != null
                        && ball3 != DetectedColor.NONE && ball3 != null;
                if (sensorsValid) {
                    intake.startSimpleSort(buildPatternFromSensors(ball1, ball2, ball3), ballOrder);
                }
            } else {
                intake.startSimpleSort("PPG", ballOrder);
            }
        }

        // Ball order selection
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            ballOrder = LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            ballOrder = LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;
        }

        if (gamepad1.left_bumper) {
            intake.setCanShoot(true);
        } else if (gamepad1.right_bumper) {
            intake.setCanShoot(false);
        }


        intake.updateProx(sensors.getColor1Proximity(),
                sensors.getColor2Proximity(),
                sensors.getColor3Proximity());

        // Telemetry
        telemetry.addLine("=== SORTING TESTER ===");
        telemetry.addData("Intake State", intake.getSimpleSortState());
        telemetry.addData("Target Ball Order", ballOrder);
        telemetry.addData("canShoot", Intake.canShoot);
        telemetry.addLine("");
        telemetry.addLine("--- Color Sensors ---");
        telemetry.addData("Slot 1", ball1 != null ? ball1 : "NONE");
        telemetry.addData("Slot 2", ball2 != null ? ball2 : "NONE");
        telemetry.addData("Slot 3", ball3 != null ? ball3 : "NONE");
        telemetry.addData("Proximity 1", sensors.getColor1Proximity());
        telemetry.addData("Proximity 2", sensors.getColor2Proximity());
        telemetry.addData("Proximity 3", sensors.getColor3Proximity());
        telemetry.addData("Rotation counter", intake.simpleRotationCounter);
        telemetry.addLine("");
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("SQUARE=Sort  DPAD=BallOrder");
        telemetry.addLine("L.Bumper=canShoot ON  R.Bumper=canShoot OFF");
        telemetry.update();
    }
}
