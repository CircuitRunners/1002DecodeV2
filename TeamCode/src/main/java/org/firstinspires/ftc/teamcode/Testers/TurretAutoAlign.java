package org.firstinspires.ftc.teamcode.Testers;

import android.hardware.Sensor;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

import java.util.Locale;

@TeleOp(name = "TurretAutoAlignBlue", group = "TEST")
public class TurretAutoAlign extends OpMode {
    private Shooter turret;
    private MecanumDrive drive;
    private Sensors sensors;
    private GamepadEx player1;
    private GoBildaPinpointDriver pinpoint;
    private static final Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 36, 36, AngleUnit.DEGREES, 90);
    private static double BLUE_GOAL_X = 6.0;
    private static double BLUE_GOAL_Y = 67.5;
    private static final String HUB_NAME = "SRSHub";

    public void init() {

        player1 = new GamepadEx(gamepad1);

        turret = new Shooter(hardwareMap, telemetry);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //configurePinpoint();

        sensors = new Sensors();
        try {
            // The Sensors class handles finding and configuring the SRSHub itself
            sensors.init(hardwareMap, HUB_NAME);
            telemetry.addData("Status", "Initialization Successful!");
        } catch (Exception e) {
            telemetry.addData("Status", "FATAL ERROR during initialization!");
            telemetry.addData("Error", e.getMessage());

        }
        telemetry.update();

    }

    public void loop() {
        player1.readButtons();
        pinpoint.update();

        Pose2D currentPose = pinpoint.getPosition();

        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        double robotHeadingRad = currentPose.getHeading(AngleUnit.RADIANS);
        double turretHeadingDeg = sensors.getTurretPosition();

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(forward, strafe);
        theta = AngleUnit.normalizeRadians(theta - robotHeadingRad);

        double newForward = r * Math.sin(theta);
        double newStrafe  = r * Math.cos(theta);

        drive.drive(newForward, newStrafe, rotate);





        double robotFieldX = -currentPose.getY(DistanceUnit.INCH);
        double robotFieldY = currentPose.getX(DistanceUnit.INCH);

        double dx = BLUE_GOAL_X - robotFieldX;
        double dy = BLUE_GOAL_Y - robotFieldY;

        double angleToGoalField = Math.atan2(dx, dy);
        double turretTargetRad = AngleUnit.normalizeRadians(angleToGoalField - robotHeadingRad);

        double turretTargetDeg = Math.toDegrees(turretTargetRad);
        turret.setTurretTargetPosition(turretTargetDeg);


        turret.update(0, turretHeadingDeg, 0);

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        telemetry.addData("Pinpoint Pos: ", data);
        telemetry.addData("Field Pos: ", "X: %.3f, Y: %.3f", robotFieldX, robotFieldY);
        telemetry.addData("Turret Deg: ", turretHeadingDeg);
        telemetry.addData("Turret Target Deg", turretTargetDeg);

    }


    //CONFIGURE PINPOINT FIRST
//    private void configurePinpoint() {
//        pinpoint.setOffsets(1.91, -2.64, DistanceUnit.INCH);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.REVERSED,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED
//        );
//        pinpoint.recalibrateIMU();
//        pinpoint.setPosition(startingPose);
//    }

}
