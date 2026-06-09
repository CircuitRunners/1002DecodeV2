package org.firstinspires.ftc.teamcode.Testers.Offseason;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;

import java.util.List;

@Configurable
@TeleOp(name = "BlobDetectorHeading", group = "TEST")
public class BlobSnapToPollen extends OpMode {
    private LimelightCamera limelight;
    private List<LynxModule> allHubs;
    private GamepadEx player1;
    private MecanumDrive drive;
    private double LIMELIGHT_HEIGHT = 12.0; //inches
    private double TA_K = 3.355; //distance * sqrt(ta)
    private double LIMELIGHT_MOUNT_DEG = 5.0;
    private PIDFController pidf;
    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.1;
    private boolean toggleSwitch = false;
    private boolean driveDirectionSwitch = false;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        limelight = new LimelightCamera(hardwareMap);

        player1 = new GamepadEx(gamepad1);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setSetPoint(0);

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
            toggleSwitch = !toggleSwitch;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            driveDirectionSwitch = !driveDirectionSwitch;
        }

        if (toggleSwitch) {
            pidf.setPIDF(kP, kI, kD, kF);
            double driveOutput = pidf.calculate(getBlobPoseTx());
            if (driveDirectionSwitch) {
                drive.setPowers(-driveOutput, driveOutput, -driveOutput, driveOutput);
            } else {
                drive.setPowers(driveOutput, -driveOutput, driveOutput, -driveOutput);
            }
        }

        double[] blobTxTy = limelight.getPollenDetectorResults();
        if (blobTxTy != null) {
            telemetry.addLine("~~~Limelight Pose~~~");
            telemetry.addData("Tx", blobTxTy[1]);
            telemetry.addData("Ty", -blobTxTy[0]);
            telemetry.addData("Distance From Blob", LIMELIGHT_HEIGHT / Math.tan(Math.toRadians(blobTxTy[0] + LIMELIGHT_MOUNT_DEG)));
        }

        telemetry.update();
    }

    private double getBlobPoseTx() {
        double[] blobTxTy = limelight.getPollenDetectorResults();
        if (blobTxTy == null) {return 0;}
        return blobTxTy[0];
    }


}
