package org.firstinspires.ftc.teamcode.Testers.Offseason;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Configurable
@TeleOp(name = "BlobDetectionTester", group = "TEST")
public class BlobDetectionTester extends OpMode {
    private LimelightCamera limelight;
    private List<LynxModule> allHubs;
    public static double LIMELIGHT_HEIGHT = 12.0; // inches
    public static double LIMELIGHT_MOUNT_DEG = 10.0;
    public static double DETECTOR_DISTANCE_THRESHOLD = 10;

    private boolean automatedDrive = false;
    private static final double TA_K = 3.355; // distance * sqrt(ta)
    private PathChain followBlob;
    private GamepadEx player1;
    private Follower follower;
    public static boolean usingColor = false;
    private double[] blobTxTy = {0, 0, 0};
    private double ta;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        limelight = new LimelightCamera(hardwareMap);
        player1 = new GamepadEx(gamepad1);

        telemetry.addLine("Init done");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // 1. Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // 2. Update follower and buttons every loop
        follower.update();
        Pose currentPose = follower.getPose();
        player1.readButtons();

        // 3. Toggle detection method
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            usingColor = !usingColor;
        }

        // 4. Reset pose
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

        // 5. Start automated drive on TRIANGLE press — build path once here, not every loop
        if (!follower.isBusy() && blobTxTy != null  && player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            buildFollowBlobDetectionPath(follower.getPose());
            automatedDrive = true;
        }

        // 6. Stop automated drive on CIRCLE or when follower finishes
        if (automatedDrive && (player1.wasJustPressed(GamepadKeys.Button.CIRCLE) || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // 7. Manual drive only when not in automated mode
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // robot centric
            );
        }

        // 8. Fetch limelight data for telemetry
        if (usingColor) {
            blobTxTy = limelight.getColorResults();
        } else {
            blobTxTy = limelight.getPollenDetectorResults();
        }

        telemetry.addLine("~~~Current Pose~~~");
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", currentPose.getHeading());


        if (blobTxTy != null) {
            telemetry.addLine("~~~Limelight Pose~~~");
            telemetry.addData("Tx", blobTxTy[0]);
            telemetry.addData("Ty", -blobTxTy[1]);
            telemetry.addData("Distance From Blob", LIMELIGHT_HEIGHT/Math.tan(Math.toRadians(-blobTxTy[1] + LIMELIGHT_MOUNT_DEG)));
            telemetry.addData("Blob Pose", getCloserToBlobPoseDetector(currentPose));
        }

        telemetry.addData("Detection Method", usingColor ? "Color Pipeline" : "Detector Pipeline");
        telemetry.addLine("");
        telemetry.addData("Automated Drive", automatedDrive ? "indubidetley" : "nah");
        telemetry.update();
    }


    private Pose getCloserToBlobPoseDetector(Pose currentPose) {
        double[] blobTxTyTa = limelight.getPollenDetectorResults();
        if (blobTxTyTa == null) {return currentPose;}
        double tx = blobTxTyTa[0];
        double ty = -blobTxTyTa[1];
        double ta = blobTxTyTa[2];

//        double distanceFromBlob = Math.cos(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG)) * (TA_K / Math.sqrt(ta));
        if (ty + LIMELIGHT_MOUNT_DEG < 2) return currentPose;
        double distanceFromBlob = LIMELIGHT_HEIGHT/Math.tan(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG));
        double blobAngle = currentPose.getHeading() - Math.toRadians(tx);
        double newX = currentPose.getX() + Math.cos(blobAngle) * distanceFromBlob;
        double newY = currentPose.getY() + Math.sin(blobAngle) * distanceFromBlob;
        double newHeading = currentPose.getHeading() - Math.toRadians(tx);

        if (Math.abs(newX) > 120 || Math.abs(newY) > 120) return currentPose;

        // Reject poses that would make the robot drive backward: the movement vector
        // (current -> new) must point in the same direction the robot is facing. Even with a
        // correct heading, a negative distanceFromBlob flips x/y behind the robot, so check the
        // displacement against the heading's forward vector rather than trusting the heading alone.
        double moveX = newX - currentPose.getX();
        double moveY = newY - currentPose.getY();
        double forwardDot = moveX * Math.cos(newHeading) + moveY * Math.sin(newHeading);
        if (forwardDot < 0) return currentPose;

        return new Pose(newX, newY, newHeading);
    }

    private void buildFollowBlobDetectionPath(Pose currentPose) {
        Pose targetPose = getCloserToBlobPoseDetector(currentPose);
        Pose approachPose = getApproachPose(currentPose, targetPose, DETECTOR_DISTANCE_THRESHOLD);
        if (Math.hypot(approachPose.getX() - currentPose.getX(), approachPose.getY() - currentPose.getY()) < DETECTOR_DISTANCE_THRESHOLD) return;

        followBlob = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, approachPose))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();

        follower.followPath(followBlob, true);
    }

    private Pose getApproachPose(Pose currentPose, Pose targetPose, double desiredDistance) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distanceToTarget = Math.hypot(dx, dy);

        if (distanceToTarget <= desiredDistance) return currentPose;

        double ux = dx / distanceToTarget;
        double uy = dy / distanceToTarget;
        double travelDistance = distanceToTarget - desiredDistance;

        double newX = currentPose.getX() + ux * travelDistance;
        double newY = currentPose.getY() + uy * travelDistance;
        double heading = Math.atan2(dy, dx);

        return new Pose(newX, newY, targetPose.getHeading());
    }
}