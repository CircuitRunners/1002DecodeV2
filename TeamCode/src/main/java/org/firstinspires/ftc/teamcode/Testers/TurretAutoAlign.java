package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

@Configurable
@TeleOp(name = "TurretAutoAlignLimelightTester", group = "TEST")
public class TurretAutoAlign extends OpMode {
    private Shooter turret;
    private MecanumDrive drive;
    private Sensors sensors;
    private GamepadEx player1;
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;
    private LimelightCamera limelight;
    private static final double METERS_TO_INCH = 39.37;

    private static final Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 36, 36, AngleUnit.DEGREES, 90);
    private static double RED_GOAL_X = 126;
    private static double GOAL_Y = 137;
    private boolean isRedAlliance = true;
    private boolean preselectFromAuto = false;

    public static double limelightTurretScale = 1.0;
    public static double limelightTurretTolerance = 0.5; //deg
    private static final String HUB_NAME = "SRSHub";

    @Override
    public void init() {

        player1 = new GamepadEx(gamepad1);

        turret = new Shooter(hardwareMap, telemetry,false);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());
        follower.update();

        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        if(Poses.getAlliance() !=null){
            if(Poses.getAlliance() == Poses.Alliance.RED){
                isRedAlliance = true;
                preselectFromAuto = true;
            }
            else {
                isRedAlliance = false;
                preselectFromAuto = true;
            }
        }
        else {
            isRedAlliance = false;
            preselectFromAuto = false;
        }

    }

    public void init_loop() {
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            isRedAlliance = !isRedAlliance;
        }
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Preselect", preselectFromAuto ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();
        pinpoint.update();
        follower.update();
        Pose currentPose = follower.getPose();
        Pose2D currentPinpointPose = pinpoint.getPosition();
        double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
        double robotVelX = pinpoint.getVelX(DistanceUnit.METER);
        double robotVelY = pinpoint.getVelY(DistanceUnit.METER);
        double currentTurretAngle = turret.getCurrentTurretPosition();

        turret.update(currentTurretAngle);


        handleDriving(currentPose);
        handleInputOverrides();
        //updateTurretWithAprilTag();


        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );

        telemetry.addData("Position", followerData);
        telemetry.addData("Field Pos: ", "X: %.3f, Y: %.3f", currentPose.getX(),currentPose.getY());
        telemetry.addData("Turret Angle: ", currentTurretAngle);
        telemetry.addData("tx",limelight.limelightCamera.getLatestResult().getTx());
        telemetry.addData("ty",limelight.limelightCamera.getLatestResult().getTy());
        //telemetry.addData("Turret Deg: ", turretHeadingDeg);

    }


    //CONFIGURE PINPOINT FIRST
    private void configurePinpoint() {
        pinpoint.setOffsets(136.603, 59.24, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }
    private void handleDriving(Pose pose) {
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void handleInputOverrides() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) updateCoordinatesWithAprilTag();
    }
    public void updateCoordinatesWithAprilTag() {
        limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();
        if (result != null && result.isValid()) {
            Pose3D mt1Pose = result.getBotpose();
            if (mt1Pose != null) {
                double finalX = (mt1Pose.getPosition().y * METERS_TO_INCH) + 72.0;
                double finalY = (-mt1Pose.getPosition().x * METERS_TO_INCH) + 72.0;
                follower.setPose(new Pose(finalX, finalY, follower.getHeading()));
                gamepad1.rumble(500);
            }
        }
    }

    public void updateTurretWithAprilTag() {
        limelight.limelightCamera.pipelineSwitch(5);
        LLResult result = limelight.getResult();
        if (result != null && result.isValid()) {
            double error = limelight.updateError();
            if (Math.abs(error) < limelightTurretTolerance) return;
            double currentTurretAngle = turret.getCurrentTurretPosition();
            double newTarget = currentTurretAngle +( error * limelightTurretScale);
            newTarget = (newTarget % 360 + 360) % 360;
            turret.setTurretTargetPosition(newTarget);

        }

    }

}
