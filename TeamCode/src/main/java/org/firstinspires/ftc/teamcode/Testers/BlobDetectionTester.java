package org.firstinspires.ftc.teamcode.Testers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
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
    private double TA_K = 3.355; //distance * sqrt(ta)
    private PathChain followBlob;
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

        limelight = new LimelightCamera(hardwareMap);

        player1 = new GamepadEx(gamepad1);

        telemetry.addLine("Init done");
        telemetry.update();

    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        player1.readButtons();
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            usingColor = !usingColor;
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
        telemetry.update();






    }

    private Pose getCloserToBlobPose(Pose currentPose) {
        double[] blobTxTy = limelight.getColorResults();
        if (blobTxTy == null) {return currentPose;}
        double tx = blobTxTy[0];
        double ty = -blobTxTy[1];

        double distanceFromBlob = LIMELIGHT_HEIGHT / Math.tan(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG));
        double newX = currentPose.getX() + Math.cos(Math.toRadians(tx)) * distanceFromBlob;
        double newY = currentPose.getY() + Math.sin(Math.toRadians(tx)) * distanceFromBlob;

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

    private Pose getCloserToBlobPoseDetector(Pose currentPose) {
        double[] blobTxTyTa = limelight.getPollenDetectorResults();
        if (blobTxTyTa == null) {return currentPose;}
        double tx = blobTxTyTa[0];
        double ty = -blobTxTyTa[1];
        double ta = blobTxTyTa[2];

        double distanceFromBlob = Math.cos(Math.toRadians(ty + LIMELIGHT_MOUNT_DEG)) * (TA_K / Math.sqrt(ta));
        double newX = currentPose.getX() + Math.cos(Math.toRadians(tx)) * distanceFromBlob;
        double newY = currentPose.getY() + Math.sin(Math.toRadians(tx)) * distanceFromBlob;

        return new Pose(newX, newY, currentPose.getHeading() + Math.toRadians(tx));
    }

}
