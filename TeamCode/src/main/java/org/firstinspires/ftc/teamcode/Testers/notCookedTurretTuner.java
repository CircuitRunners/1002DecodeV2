package org.firstinspires.ftc.teamcode.Testers;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import java.util.List;

@Configurable
@TeleOp(name = "UNCOOKED TurretPIDFTuner", group = "TEST")
public class notCookedTurretTuner extends OpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.007;
    public static double kI = 0.000;
    public static double kD = 0.000;
    public static double kF = 0.0; // Usually 0 for horizontal turrets
    public static double targetAngle = 180;
    public static double maxPower = 1.0;
    public static double tolerance = 2.0; // Degrees

    // ===== Hardware & State =====
    private DcMotorEx turret;
    private PIDFController pidf;
    private Sensors sensors = new Sensors();

    private ElapsedTime loopTimer = new ElapsedTime();
    //private ElapsedTime settleTimer = new ElapsedTime();
    private boolean isMoving = false;
    private static final String HUB_NAME = "SRSHub";

    @Override
    public void init() {
        // Bulk reads for performance
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // FIX: Use WITHOUT_ENCODER so the custom PIDF has full control
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            sensors.init(hardwareMap, HUB_NAME);
            telemetry.addData("Status", "SRS Initialization Successful!");
        } catch (Exception e) {
            telemetry.addData("Status", "FATAL ERROR during SRS initialization!");
        }

        pidf = new PIDFController(kP, kI, kD, kF);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear cache for bulk reads
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        sensors.update();
        double currentAngle = sensors.getSketchTurretPosition();

        // FIX: Re-sync gains every loop so Dashboard tuning works live
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setSetPoint(targetAngle);

        // Compute PID output
        double turretOutput = pidf.calculate(currentAngle);
        turretOutput = Range.clip(turretOutput, -maxPower, maxPower);
        turret.setPower(turretOutput);

        // Settle logic
        //double error = targetAngle - currentAngle;
//        if (Math.abs(error) > tolerance) {
//            if (!isMoving) {
//                isMoving = true;
//                settleTimer.reset();
//            }
//        } else {
//            isMoving = false;
//        }

        // --- Telemetry (One update call at the end) ---
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Current Angle", "%.2f", currentAngle);
        //telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Output Power", "%.2f", turretOutput);

//        if (!isMoving) {
//            telemetry.addData("Status", "Settled");
//            telemetry.addData("Settle Time (ms)", settleTimer.milliseconds());
//        } else {
//            telemetry.addData("Status", "Moving...");
//        }

        telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
        telemetry.update();

        loopTimer.reset();
    }
}