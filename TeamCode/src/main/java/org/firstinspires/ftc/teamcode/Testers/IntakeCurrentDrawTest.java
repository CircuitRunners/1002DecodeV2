package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;
@Disabled
@TeleOp
public class IntakeCurrentDrawTest  extends OpMode {

    private GamepadEx player1;
    private Intake intake;

    private boolean doOuttake = false;


    private LimelightCamera.BallOrder ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap, telemetry);


//

        intake.setCanShoot(false);
        telemetry.addLine("init done");
        telemetry.update();
    }
//hi

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        player1.readButtons();


        telemetry.addLine("");
        telemetry.addData("Status", "current motor 1: " + intake.motor1CurrentDraw());
        telemetry.addLine("");
        telemetry.addData("Status", "current motor 2: " + intake.motor2CurrentDraw());


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


        // Hold Cross (A) to Intake
//        telemetry.addData("Ball 1: ", ball1);
//        telemetry.addData("Ball 2: ", ball2);
//        telemetry.addData("Ball 3: ", ball3);

        intake.update(false, ballOrder, DetectedColor.GREEN, DetectedColor.GREEN, DetectedColor.PURPLE);

        if (gamepad1.right_trigger > 0.2) {
            intake.doIntake();
            telemetry.addData("Status", "Intaking...");
        }

        if (
                //intake.motor1CurrentDraw() >= 1625 &&
                        intake.motor1CurrentDraw() >= 3400){
            doOuttake = true;
            intake.doOuttake();
        }
        if (doOuttake && intake.motor1CurrentDraw() < 2900){
            intake.doIntakeHalt();
            doOuttake = false;
        }
        //
        // Hold Circle (B) to Cycle



        //intake.doSortingTelemetry(ball1,ball2,ball3,ballOrder, shooter.isBeamBroken());

        telemetry.update();

    }

    @Override
    public void stop(){
//        if (colorSensorThread != null) {
//            colorSensorThread.interrupt();
//        }
    }
}
