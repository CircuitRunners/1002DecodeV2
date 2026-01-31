package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

@Disabled
@TeleOp(name = "ShooterTester", group = "TEST")
public class ShooterTester extends OpMode {

    //private MecanumDrive drive;
    private Shooter shooter;
    private GoBildaPinpointDriver pinpoint;
    private GamepadEx player1;
    private LimelightCamera limelight;
    private Sensors sensors = new Sensors();
    private boolean areBothBeamsBroken;
    private int totalInventory = 0;
    private double robotFieldYawDegrees;
    private Shooter.TurretMode turretMode = Shooter.TurretMode.FIELD_CENTRIC;
    private static final double METERS_TO_INCH = 39.37;
    private static final double BLUE_GOAL_X_INCHES = 6.0;
    private static final double BLUE_GOAL_Y_INCHES = 68.5;
    private static final double TICKS_PER_REV = 537.7;

    private enum INTAKE_STATUS {INTAKING, CYCLING, IDLE, TRANSFERING}

    ;
    INTAKE_STATUS intakeStatus = INTAKE_STATUS.IDLE;

    @Override
    public void init() {
        telemetry.addLine("Intializing...");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

//        drive = new MecanumDrive();
//        drive.init(hardwareMap);

        shooter = new Shooter(hardwareMap, telemetry);

        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //configurePinpoint();

        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();

        telemetry.addLine("Ready!");
        telemetry.update();
    }


    @Override
    public void loop() {
        player1.readButtons();
        pinpoint.update();


//        double forward = player1.getLeftY();
//        double strafe = player1.getLeftX();
//        double rotate = player1.getRightX();
//
//        drive.drive(forward, strafe, rotate);

        robotFieldYawDegrees = pinpoint.getHeading(AngleUnit.DEGREES);

//        shooter.setShooterTarget(
//                pinpoint.getPosX(DistanceUnit.INCH),
//                pinpoint.getPosY(DistanceUnit.INCH),
//                BLUE_GOAL_X_INCHES,
//                BLUE_GOAL_Y_INCHES, pinpoint.getVelX(DistanceUnit.INCH),pinpoint.getVelY(DistanceUnit.INCH)
//        );

        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            updateCoordinates();

            double desiredTurretHeading = 0.0; //field centric auto align
//            shooter.setShooterTarget(
//                    pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
//                    BLUE_GOAL_X_INCHES, BLUE_GOAL_Y_INCHES);
//            //blue goal only


        }

        shooter.update(sensors.getFlywheelVelo(), shooter.getCurrentTurretPosition());


        telemetry.addData("Pinpoint X Inches: ", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y Inches: ", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading Degrees: ", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Flywheel Velocity (RPM): ", sensors.getFlywheelVelo() / TICKS_PER_REV * 60);
        telemetry.addData("Turret Position Degrees: ", shooter.getCurrentTurretPosition());
        //telemetry.addData("Hood Position Degrees: ", shooter.getHoodServoPositionInDegrees());
        telemetry.addLine("");
        telemetry.addLine("===== Auto Shot =====");
        telemetry.addData("Flywheel Target (ticks)", shooter.getTargetFLywheelVelo());
        telemetry.addData("Hood Target (deg)", shooter.getCurrentRequiredHoodAngle());
        //telemetry.addData("Time of Flight (s)", shooter.getCurrentRequiredInAirTOF());


        telemetry.update();

    }

    public void updateCoordinates() {
        //updates the orientation of robot for limelight camera's usage
        limelight.limelightCamera.updateRobotOrientation(pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        //limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();


        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    Pose3D mt1Pose = result.getBotpose();
                    if (mt1Pose != null) {
                        //gets raw position from limelight in m
                        double llX_m = mt1Pose.getPosition().x;
                        double llY_m = mt1Pose.getPosition().y;
                        //converts position to inches
                        double llX_in = llX_m * METERS_TO_INCH;
                        double llY_in = llY_m * METERS_TO_INCH;
                        //rotates limelight points 90 degrees counterclockwise
                        double llX_in_rotated = llY_in;
                        double llY_in_rotated = -llX_in;
                        //shifts position to bottom left corner field origin for pedro pathing use
//                        double llX_in_shifted = llX_in_rotated + 72.0;
//                        double llY_in_shifted = llY_in_rotated + 72.0;

                        double currentHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);  // keep heading from Pinpoint
                        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, llX_in_rotated, llY_in_rotated, AngleUnit.RADIANS, currentHeadingRad));
                        telemetry.addData("New Pose From Apriltag:", "X: %.2f in, Y: %.2f in, H: %.1f°", llX_in_rotated, llY_in_rotated, Math.toDegrees(currentHeadingRad));
                    }
                    break;
                }
            }
        }


    }
}

// CONFIGURE PINPOINT FIRST
//    private void configurePinpoint() {
//        pinpoint.setOffsets(1.91 , -2.64, DistanceUnit.INCH); //NEEDS TO BE MEASURED
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.REVERSED,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED
//        );
//        pinpoint.recalibrateIMU();
//    }
//}
