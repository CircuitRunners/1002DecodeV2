package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static double kF = 0.0;
    public static double targetAngle = 180;
    public static double maxPower = 1.0;
    public static double minOutput = 0.05;
    public static double tolerance = 2.0;

    // Toggle this BEFORE hitting Init on the Driver Station
    public static boolean isReversed = false;

    // ===== Hardware & State =====
    private DcMotorEx turret;
    private PIDFController pidf;
    private Sensors sensors = new Sensors();
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final String HUB_NAME = "SRSHub";

    @Override
    public void init() {
        // 1. Bulk Read Setup
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // 2. Motor Setup
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Direction Logic (captured at Init)
        turret.setDirection(isReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // 4. Sensor/Controller Setup
        try {
            sensors.init(hardwareMap, HUB_NAME);
            telemetry.addData("Status", "SRS Initialization Successful!");
        } catch (Exception e) {
            telemetry.addData("Status", "FATAL ERROR: " + e.getMessage());
        }

        pidf = new PIDFController(kP, kI, kD, kF);
        //pidf.setMinimumOutput(minOutput);

        telemetry.addData("Mode", isReversed ? "REVERSED" : "FORWARD");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear bulk cache for fresh data
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        sensors.update();
        double currentAngle = sensors.getSketchTurretPosition();

        // Update PID coefficients & target live from Dashboard
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setSetPoint(targetAngle);


        // Calculation
        double turretOutput = pidf.calculate(currentAngle);
        turretOutput = Range.clip(turretOutput, -maxPower, maxPower);
        turret.setPower(turretOutput);

        // Telemetry
        double error = targetAngle - currentAngle;
        telemetry.addData("Target", targetAngle);
        telemetry.addData("Actual", "%.2f", currentAngle);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Power", "%.2f", turretOutput);
        telemetry.addData("Loop ms", loopTimer.milliseconds());
        telemetry.update();

        loopTimer.reset();
    }
}