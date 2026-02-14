package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;

@TeleOp( group = "TEST")
public class SigmaIntakeTest extends OpMode {

    private GamepadEx player1;
    private Intake intake;
    private Sensors sensors;
    private Shooter shooter;
    private Thread colorSensorThread;
    private LimelightCamera.BallOrder ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap, telemetry);
        sensors = new Sensors();
        shooter = new Shooter(hardwareMap,telemetry,false);
        sensors.init(hardwareMap, "SRSHub");

//        try {
//            // The Sensors class handles finding and configuring the SRSHub itself
//
//            telemetry.addData("Status", "Initialization Successful!");
//        } catch (Exception e) {
//            telemetry.addData("Status", "FATAL ERROR during initialization!");
//            telemetry.addData("Error", e.getMessage());
//
//        }

        intake.setCanShoot(false);
        telemetry.addLine("init done");
        telemetry.update();
    }
//hi

    @Override
    public void start() {
        colorSensorThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                sensors.update();
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    break;
                }
            }
        });

        colorSensorThread.start();
    }

    @Override
    public void loop() {
        player1.readButtons();

        DetectedColor ball1 = sensors.getDetectedColor(sensors.colorSensor1);
        DetectedColor ball2 = sensors.getDetectedColor(sensors.colorSensor2);
        DetectedColor ball3 = sensors.getDetectedColor(sensors.colorSensor3);
        telemetry.addData("Ball 1: ", ball1);
        telemetry.addData("Ball 2: ", ball2);
        telemetry.addData("Ball 3: ", ball3);
        telemetry.addLine("");

//        NormalizedRGBA rgba1 = sensors.colorSensor1.getNormalizedColors();
//        float r1 = rgba1.red * 10000;
//        float g1 = rgba1.green * 10000;
//        float b1 = rgba1.blue * 10000;
//        NormalizedRGBA rgba2 = sensors.colorSensor2.getNormalizedColors();
//        float r2 = rgba2.red * 10000;
//        float g2 = rgba2.green * 10000;
//        float b2 = rgba2.blue * 10000;
//        NormalizedRGBA rgba3 = sensors.colorSensor3.getNormalizedColors();
//        float r3 = rgba3.red * 10000;
//        float g3 = rgba3.green * 10000;
//        float b3 = rgba3.blue * 100000;

//        telemetry.addData("1r: ", r1);
//        telemetry.addData("1g: ", g1);
//        telemetry.addData("1b: ", b1);
//        telemetry.addLine("");
//        telemetry.addData("2r: ", r2);
//        telemetry.addData("2g: ", g2);
//        telemetry.addData("2b: ", b2);
//        telemetry.addLine("");
//
//        telemetry.addData("3r: ", r3);
//        telemetry.addData("3g: ", g3);
//        telemetry.addData("3b: ", b3);
        telemetry.addLine("");
        telemetry.update();


        // Hold Cross (A) to Intake
//        telemetry.addData("Ball 1: ", ball1);
//        telemetry.addData("Ball 2: ", ball2);
//        telemetry.addData("Ball 3: ", ball3);

        intake.update(shooter.isBeamBroken(), ballOrder, ball1, ball2, ball3);
        if (gamepad1.square) {
            intake.prepareAndStartSort();
        }
        else if (gamepad1.dpad_up){
            ballOrder = LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE;
        } else if (gamepad1.dpad_down) {
            ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;
        }
        else if (gamepad1.dpad_left) {
            ballOrder = LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;
        }


         else if (gamepad1.left_bumper){
            intake.setCanShoot(true);
        }
        else if (gamepad1.right_bumper){
            intake.setCanShoot(false);
        }
        else if (player1.isDown(GamepadKeys.Button.CROSS)) {
            intake.doIntake();
            telemetry.addData("Status", "Intaking...");
        }
        // Hold Circle (B) to Cycle
        else if (player1.isDown(GamepadKeys.Button.CIRCLE)) {
            intake.doCycle();
            telemetry.addData("Status", "Cycling...");
        }

        else if (player1.isDown(GamepadKeys.Button.TRIANGLE)){
            intake.doTestShooter();
        }
        // Otherwise, stop
        else if (player1.isDown(GamepadKeys.Button.DPAD_UP)){
            intake.doIntakeHalt();
            telemetry.addData("Status", "Idle");
        }

        //intake.doSortingTelemetry(ball1,ball2,ball3,ballOrder, shooter.isBeamBroken());

    }

    @Override
    public void stop(){
        if (colorSensorThread != null) {
            colorSensorThread.interrupt();
        }
    }
}
