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

@TeleOp(name = "SortTester", group = "TEST")
public class IntakeTester extends OpMode {

    private GamepadEx player1;
    private Intake intake;
    private Sensors sensors;
    private static final String HUB_NAME = "SRSHub";
    private LimelightCamera.BallOrder ballOrder = LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE;
    private enum INTAKE_STATUS {INTAKING, IDLE, SORTING, TESTING};
    private INTAKE_STATUS intakeStatus = INTAKE_STATUS.IDLE;
    private enum TESTING_SERVO {TRANSFER_DIRECTION, TRANSFER_SERVO, GATE}
    public TESTING_SERVO testingStatus = TESTING_SERVO.TRANSFER_DIRECTION;
    private double TRANSFER_DIRECTION_TRANSFER_POS = 0.75;
    private double TRANSFER_DIRECTION_CYCLE_POS = 0.3;
    private double GATE_OPEN = 0.7;
    private double GATE_CLOSE = 0.3;
    private double TRANSFER_ON = 0.7;
    private double TRANSFER_OFF = 0.3;
    private Shooter shooter;


    @Override
    public void init() {

        player1 = new GamepadEx(gamepad1);

        intake = new Intake(hardwareMap, telemetry);

        sensors = new Sensors();
        shooter = new Shooter(hardwareMap,telemetry);
        try {
            // The Sensors class handles finding and configuring the SRSHub itself
            sensors.init(hardwareMap, HUB_NAME);
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

        //boolean beamBreak = shooter.isBeamBroken();

        //intake
        if (intakeStatus != INTAKE_STATUS.TESTING) {
            if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                switch (intakeStatus) {
                    case IDLE:
                        intakeStatus = INTAKE_STATUS.INTAKING;
                        break;

                    case INTAKING:
                        intakeStatus = INTAKE_STATUS.IDLE;
                        break;

                }
            }

            //sort
            if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                switch (intakeStatus) {
                    case IDLE:
                        intakeStatus = INTAKE_STATUS.SORTING;
                        break;

                    case INTAKING:
                        intakeStatus = INTAKE_STATUS.SORTING;
                        break;

                    case SORTING:
                        intakeStatus = INTAKE_STATUS.IDLE;

                }
            }

            if (gamepad2.square){
                intake.sort(shooter.isBeamBroken(), ballOrder, ball1, ball2, ball3);
            }

            if (gamepad2.left_bumper){
                intake.setCanShoot(true);
            }
            if (gamepad2.right_bumper){
                intake.setCanShoot(false);
            }

            switch (intakeStatus) {
                case INTAKING:
                    intake.intake();
                    if (ball1 != null) { //whichever color sensor is first
                        intake.cycle();
                    }
                    break;

                case SORTING:
//                    intake.sort(shooter.isBeamBroken(), ballOrder, ball1, ball2, ball3);
                    break;

                case IDLE:
                    intake.intakeMotorIdle();
                    break;

            }

            if (ball1 == DetectedColor.GREEN){
                telemetry.addLine("Detected Color slot 1 : GREEN FN");
            }
            else  if (ball1 == DetectedColor.PURPLE){
                telemetry.addLine("Detected Color slot 1 : Purple");
            }

            if (ball2 == DetectedColor.GREEN){
                telemetry.addLine("Detected Color slot 2 : GREEN FN");
            }
            else  if (ball2 == DetectedColor.PURPLE){
                telemetry.addLine("Detected Color slot 2 : Purple");
            }
            if (ball3 == DetectedColor.GREEN){
                telemetry.addLine("Detected Color slot 3 : GREEN FN");
            }
            else  if (ball3 == DetectedColor.PURPLE){
                telemetry.addLine("Detected Color slot 3 : Purple");
            }
           // telemetry.addData("Beambreak: ", beamBreak);
            telemetry.addLine("");
            telemetry.update();
        }





        //servo testing
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            if (intakeStatus != INTAKE_STATUS.TESTING) {
                intakeStatus = INTAKE_STATUS.TESTING;
            } else {
                intakeStatus = INTAKE_STATUS.IDLE;
            }
        }

        if (intakeStatus == INTAKE_STATUS.TESTING) {

            if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
                testingStatus = TESTING_SERVO.TRANSFER_DIRECTION;
            } else if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                testingStatus = TESTING_SERVO.TRANSFER_SERVO;
            } else if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                testingStatus = TESTING_SERVO.GATE;
            }

            switch (testingStatus) {
                case TRANSFER_DIRECTION:
                    if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        TRANSFER_DIRECTION_TRANSFER_POS += 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        TRANSFER_DIRECTION_TRANSFER_POS -= 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        TRANSFER_DIRECTION_CYCLE_POS += 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        TRANSFER_DIRECTION_CYCLE_POS -= 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        intake.setDirectionSwitcherPosition(TRANSFER_DIRECTION_TRANSFER_POS);
                    } else if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.setDirectionSwitcherPosition(TRANSFER_DIRECTION_CYCLE_POS);
                    }
                    break;

                case TRANSFER_SERVO:
                    if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        TRANSFER_ON += 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        TRANSFER_ON -= 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        TRANSFER_OFF += 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        TRANSFER_OFF -= 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        intake.setPowerTransmitionPosition(TRANSFER_ON);
                    } else if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.setPowerTransmitionPosition(TRANSFER_OFF);
                    }
                    break;

                case GATE:
                    if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        GATE_OPEN += 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        GATE_OPEN -= 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        GATE_CLOSE += 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        GATE_CLOSE -= 0.05;
                    } else if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        intake.setGatePositon(GATE_CLOSE);
                    } else if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.setGatePositon(GATE_OPEN);
                    }
                    break;
            }

            telemetry.addLine("TESTING SERVO POSITION");
            telemetry.addData("Intake Status: ", intakeStatus);
            telemetry.addData("Testing Status: ", testingStatus);
            telemetry.addLine("");
            telemetry.addData("Transfer Direction Transfer: ", TRANSFER_DIRECTION_TRANSFER_POS);
            telemetry.addData("Transfer Direction Cycle: ", TRANSFER_DIRECTION_CYCLE_POS);
            telemetry.addLine("");
            telemetry.addData("Transfer On: ", TRANSFER_ON);
            telemetry.addData("Transfer Off: ", TRANSFER_OFF);
            telemetry.addLine("");
            telemetry.addData("Gate Open: ", GATE_OPEN);
            telemetry.addData("Gate Close: ", GATE_CLOSE);
            telemetry.update();
        }



    }

}
