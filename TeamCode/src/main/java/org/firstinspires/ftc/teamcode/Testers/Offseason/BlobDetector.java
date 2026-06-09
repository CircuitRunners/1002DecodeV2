package org.firstinspires.ftc.teamcode.Testers.Offseason;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

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
    private double TA_K = 3.355; //distance * sqrt(ta)
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

        double[] blobTxTy = limelight.getPollenDetectorResults();

        if (!follower.isBusy() && blobTxTy != null) {
            buildBlobDetectionPath(currentPose);
        }

        telemetry.addLine("~~~Current Pose~~~");
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", currentPose.getHeading());


        if (blobTxTy != null) {
            telemetry.addLine("~~~Limelight Pose~~~");
            telemetry.addData("Tx", blobTxTy[0]);
            telemetry.addData("Ty", -blobTxTy[1]);
            telemetry.addData("Distance From Blob", Math.cos(Math.toRadians(-blobTxTy[1] + LIMELIGHT_MOUNT_DEG)) * (TA_K / Math.sqrt(blobTxTy[2])));
            telemetry.addData("Blob Pose", getCloserToBlobPoseDetector(currentPose));
        }

        telemetry.update();






    }

    private Pose getCloserToBlobPoseDetector(Pose currentPose) {
        double[] blobTxTyTa = limelight.getPollenDetectorResults();
        if (blobTxTyTa == null) {return currentPose;}
        double tx = blobTxTyTa[0];
        double ty = -blobTxTyTa[1];
        double ta = blobTxTyTa[2];

        double distanceFromBlob = Math.cos(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG)) * (TA_K / Math.sqrt(ta));
        double blobAngle = currentPose.getHeading() + Math.toRadians(tx);
        double newX = currentPose.getX() + Math.cos(blobAngle) * distanceFromBlob;
        double newY = currentPose.getY() + Math.sin(blobAngle) * distanceFromBlob;

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

    private void buildBlobDetectionPath(Pose currentPose) {
        Pose targetPose = getCloserToBlobPoseDetector(currentPose);
        if (Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY()) < 2) return;

        followBlob = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();

        follower.followPath(followBlob, true);
    }
}
