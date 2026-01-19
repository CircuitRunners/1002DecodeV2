package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    private LimelightCamera.BallOrder ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap, telemetry);
        sensors = new Sensors();
        shooter = new Shooter(hardwareMap,telemetry);

        try {
            // The Sensors class handles finding and configuring the SRSHub itself
            sensors.init(hardwareMap, "SRSHub");
            telemetry.addData("Status", "Initialization Successful!");
        } catch (Exception e) {
            telemetry.addData("Status", "FATAL ERROR during initialization!");
            telemetry.addData("Error", e.getMessage());

        }

        intake.setCanShoot(false);
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();

        sensors.update();

        DetectedColor ball1 = sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green());
        DetectedColor ball2 = sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green());
        DetectedColor ball3 = sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green());
        // Hold Cross (A) to Intake

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
        // Otherwise, stop
        else {
            intake.doIntakeHalt();
            telemetry.addData("Status", "Idle");
        }

        intake.doSortingTelemetry(ball1,ball2,ball3,ballOrder, shooter.isBeamBroken());

        telemetry.update();
    }
}