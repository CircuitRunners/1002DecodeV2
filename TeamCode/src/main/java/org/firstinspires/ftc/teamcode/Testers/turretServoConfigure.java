package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

@TeleOp(name = "Configure Turret Servo", group = "Test")
@Configurable
public class turretServoConfigure extends OpMode {

    private GamepadEx player1;
    private NewShooter turret;
    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;

    public static double turret1Min = 0.0;
    public static double turret1Max = 1.0;
    public static double turret2Min = 0.0;
    public static double turret2Max = 1.0;
    public static boolean isRedAlliance = true;
    private boolean turretOn = false;
    public static double redGoalX = 127.0;
    public static double blueGoalX = 13.0;
    public static double goalY = 132.0;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);

        turret = new NewShooter(hardwareMap, telemetry,false);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());
        follower.update();

        telemetry.addLine("Ready");
        telemetry.update();

    }

    @Override
    public void loop() {
        follower.update();
        player1.readButtons();
        turret.turret1MinRange = turret1Min;
        turret.turret1MaxRange = turret1Max;
        turret.turret2MinRange = turret2Min;
        turret.turret2MaxRange = turret2Max;

        Pose currentPose = follower.getPose();
        double currentHeading = Math.toDegrees(currentPose.getHeading());

        handleDriving(currentPose);
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(new Pose(72, 72, Math.toRadians(90)));


        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            turretOn = !turretOn;
        }

        if (turretOn) {
            double requiredFieldYaw;
            if (isRedAlliance) {
                requiredFieldYaw = NewShooter.calculateAutoAlignYaw(currentPose.getX(), currentPose.getY(), redGoalX, goalY, true);
            }
            else {
                requiredFieldYaw = NewShooter.calculateAutoAlignYaw(currentPose.getX(), currentPose.getY(), blueGoalX, goalY, false);
            }

            turret.setTurretTarget(requiredFieldYaw, NewShooter.TurretMode.AUTO_ALIGN, currentHeading, 0);
        }

        turret.update(turret.getCurrentTurretPosition());

        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );

        telemetry.addData("Current Pose", followerData);
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addLine("");
        telemetry.addData("Turret Position", turret.getCurrentTurretPosition());
        telemetry.addData("Turret Target", turret.getTurretTargetPosition());
        telemetry.addData("Turret On?", turretOn);
        telemetry.update();

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






}
