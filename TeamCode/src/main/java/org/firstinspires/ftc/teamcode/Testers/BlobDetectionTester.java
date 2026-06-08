package org.firstinspires.ftc.teamcode.Testers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "BlobDetectionTester", group = "TEST")
public class BlobDetectionTester extends OpMode {
    private LimelightCamera limelight;
    private List<LynxModule> allHubs;
    private double LIMELIGHT_HEIGHT = 12.0; //inches
    private double LIMELIGHT_MOUNT_DEG = 5.0;

    boolean automatedDrive = false;
    private double TA_K = 3.355; //distance * sqrt(ta)
    private PathChain followBLob;
   // private Supplier<PathChain> followBLob;
    private GamepadEx player1;
    private Follower follower;
    private boolean usingColor = false;
    private double[] blobTxTy = {0, 0, 0};
    private double ta;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.update();

        limelight = new LimelightCamera(hardwareMap);

        player1 = new GamepadEx(gamepad1);

        telemetry.addLine("Init done");
        telemetry.update();

    }

    @Override
    public void start() {
     follower.startTeleOpDrive();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        follower.update();

        followBLob =  follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose,usingColor? getCloserToBlobPose(follower.getPose()) : getCloserToBlobPoseDetector(follower.getPose()))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();



        player1.readButtons();
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            usingColor = !usingColor;
        }


        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );
        }

        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE))
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));

        if (gamepad1.triangleWasPressed()) {
            follower.followPath(followBLob);
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.circleWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (usingColor) {
            blobTxTy = limelight.getColorResults();
        } else {
            blobTxTy = limelight.getPollenDetectorResults();
        }
        if (blobTxTy != null) {
            ta = blobTxTy[2];
            telemetry.addLine("~~~Limelight Pose~~~");
            telemetry.addData("LimelightTx", blobTxTy[0]);
            telemetry.addData("LimelightTy", blobTxTy[1]);
            telemetry.addData("LimelightTa", ta);
//            telemetry.addData("Actual Tx", blobTxTy[1]);
//            telemetry.addData("Actual Ty", -blobTxTy[0]);
            telemetry.addData("Distance From Blob", usingColor ? LIMELIGHT_HEIGHT / Math.tan(Math.toRadians(-blobTxTy[1] + LIMELIGHT_MOUNT_DEG)) : Math.cos(Math.toRadians(-blobTxTy[1] + LIMELIGHT_MOUNT_DEG)) * (TA_K / Math.sqrt(ta)));
            if (usingColor) {
                telemetry.addData("Blob Pose", getCloserToBlobPose(follower.getPose()));
            } else {
                telemetry.addData("Blob Pose", getCloserToBlobPoseDetector(follower.getPose()));
            }
        }
        telemetry.addData("Detection Method", usingColor ? "Color Pipeline" : "Detector Pipeline");
        telemetry.addLine("");
        telemetry.addData("automated drive?", automatedDrive ? "indubidetley" : "nah");
        telemetry.update();






    }

    private Pose getCloserToBlobPose(Pose currentPose) {
        double[] blobTxTy = limelight.getColorResults();
        if (blobTxTy == null) {return currentPose;}
        double tx = blobTxTy[0];
        double ty = -blobTxTy[1];

        double distanceFromBlob = LIMELIGHT_HEIGHT / Math.tan(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG));
//        double newX = currentPose.getX() + Math.cos(Math.toRadians(tx)) * distanceFromBlob;
//        double newY = currentPose.getY() + Math.sin(Math.toRadians(tx)) * distanceFromBlob;

        double globalAngle = currentPose.getHeading()

                + Math.toRadians(tx);

        double newX = currentPose.getX()

                + distanceFromBlob * Math.cos(globalAngle);

        double newY = currentPose.getY()

                + distanceFromBlob * Math.sin(globalAngle);

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

    private Pose getCloserToBlobPoseDetector(Pose currentPose) {
        double[] blobTxTyTa = limelight.getPollenDetectorResults();
        if (blobTxTyTa == null) {return currentPose;}
        double tx = blobTxTyTa[0];
        double ty = -blobTxTyTa[1];
        double ta = blobTxTyTa[2];

        double distanceFromBlob = Math.cos(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG)) * (TA_K / Math.sqrt(ta));
//        double newX = currentPose.getX() + Math.cos(Math.toRadians(tx)) * distanceFromBlob;
//        double newY = currentPose.getY() + Math.sin(Math.toRadians(tx)) * distanceFromBlob;

        double globalAngle = currentPose.getHeading()

                + Math.toRadians(tx);

        double newX = currentPose.getX()

                + distanceFromBlob * Math.cos(globalAngle);

        double newY = currentPose.getY()

                + (distanceFromBlob * Math.sin(globalAngle));

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

    private Pose getApproachPose(Pose currentPose, Pose targetPose, double desiredDistance) {

        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        double distanceToTarget = Math.hypot(dx, dy);

        // Already inside the desired distance
        if (distanceToTarget <= desiredDistance) {
            return currentPose;
        }

        // Unit vector from robot to target
        double ux = dx / distanceToTarget;
        double uy = dy / distanceToTarget;

        // Move toward target but stop desiredDistance short
        double travelDistance = distanceToTarget - desiredDistance;

        double newX = currentPose.getX() + ux * travelDistance;
        double newY = currentPose.getY() + uy * travelDistance;

        // Face the target
        double heading = Math.atan2(dy, dx);

        return new Pose(newX, newY, heading);
    }

}
