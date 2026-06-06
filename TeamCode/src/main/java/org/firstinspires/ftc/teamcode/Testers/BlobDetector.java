package org.firstinspires.ftc.teamcode.Testers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "BlobDetector")
public class BlobDetector extends OpMode {
    private LimelightCamera limelight;
    private Follower follower;
    private List<LynxModule> allHubs;
    private GamepadEx player1;
    private boolean usingColor = false;
    private double LIMELIGHT_HEIGHT = 12.0; //inches
    private double TA_K = 33.55; //distance * sqrt(ta)
    private double LIMELIGHT_MOUNT_DEG = 5.0;
    private PathChain followBlob;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 90));

        limelight = new LimelightCamera(hardwareMap);

        telemetry.addLine("Init done");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        
        follower.update();
        Pose currentPose = follower.getPose();

        player1.readButtons();
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            usingColor = !usingColor;
        }


        if (!follower.isBusy()) {
            buildBlobDetectionPath(currentPose);
            follower.followPath(followBlob, false);
        }

        telemetry.addLine("~~~Current Pose~~~");
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", currentPose.getHeading());

        double[] blobTxTy = limelight.getColorResults();
        if (blobTxTy != null) {
            telemetry.addLine("~~~Limelight Pose~~~");
            telemetry.addData("Tx", blobTxTy[1]);
            telemetry.addData("Ty", -blobTxTy[0]);
            telemetry.addData("Distance From Blob", LIMELIGHT_HEIGHT / Math.tan(Math.toRadians(blobTxTy[0] + LIMELIGHT_MOUNT_DEG)));
            telemetry.addData("Blob Pose", getCloserToBlobPoseColor(currentPose));
        }

        telemetry.update();






    }

    private Pose getCloserToBlobPoseColor(Pose currentPose) {
        double[] blobTxTy = limelight.getColorResults();
        if (blobTxTy == null) {return currentPose;}
        double tx = blobTxTy[0];
        double ty = blobTxTy[1];

        double distanceFromBlob = LIMELIGHT_HEIGHT / Math.tan(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG));
        if (distanceFromBlob < 2) {return currentPose;}
        double newX = currentPose.getX() + Math.cos(Math.toRadians(tx)) * distanceFromBlob;
        double newY = currentPose.getY() + Math.sin(Math.toRadians(tx)) * distanceFromBlob;

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

    private Pose getCloserToBlobPoseDetector(Pose currentPose) {
        double[] blobTxTyTa = limelight.getPollenDetectorResults();
        if (blobTxTyTa == null) {return currentPose;}
        double tx = blobTxTyTa[1];
        double ty = -blobTxTyTa[0];
        double ta = blobTxTyTa[2];

        double distanceFromBlob = TA_K / Math.sqrt(ta);
        if (distanceFromBlob < 2) {return currentPose;}
        double newX = currentPose.getX() + Math.cos(Math.toRadians(tx)) * distanceFromBlob;
        double newY = currentPose.getY() + Math.sin(Math.toRadians(tx)) * distanceFromBlob;

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

    public void buildBlobDetectionPath(Pose currentPose) {
        Pose targetPose;
        if (usingColor) {
            targetPose = getCloserToBlobPoseColor(currentPose);
        } else {
            targetPose = getCloserToBlobPoseDetector(currentPose);
        }
        followBlob = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();

    }
}
