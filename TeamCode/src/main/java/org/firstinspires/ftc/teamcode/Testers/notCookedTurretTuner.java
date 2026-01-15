package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

import java.util.List;

@Configurable
@TeleOp(name = "TurretPIDFTuner_DualTelemetry", group = "TEST")
public class notCookedTurretTuner extends OpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.007;
    public static double kI = 0.000;
    public static double kD = 0.000;
    public static double kF = 0.0;
    public static double targetAngle = 180;
    public static double maxPower = 1.0;
    public static boolean isReversed = false;

    // ===== Hardware & State =====
    private DcMotorEx turret;
    private PIDFController pidf;
    private Sensors sensors = new Sensors();
    private ElapsedTime loopTimer = new ElapsedTime();
    private Shooter shooter;

    public static double cookedLoopTargetMS = 40;
   // private TelemetryManager panelsTelemetry;
    private static final String HUB_NAME = "SRSHub";

    @Override
    public void init() {
        // Initialize Panels Telemetry system
       // panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Bulk Read Setup for faster loop times
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Motor Setup
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Direction logic based on the Dashboard toggle
        turret.setDirection(isReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        shooter = new Shooter(hardwareMap,telemetry);

        // 4. Sensor Initialization
        try {
            sensors.init(hardwareMap, HUB_NAME);
            telemetry.addData("Status", "SRS Hub Connected");
        } catch (Exception e) {
            telemetry.addData("Status", "SRS Hub Error: " + e.getMessage());
        }

        // 5. PID Setup
        pidf = new PIDFController(kP, kI, kD, kF);

        telemetry.addLine("Ready to tune. Check Panels UI for graphs.");
       // panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        // Clear bulk cache at the start of every loop
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        // Update sensor data
        sensors.update();
        double currentAngle = shooter.getCurrentTurretPosition();

        // Update PID coefficients and target live from Dashboard
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setSetPoint(targetAngle);

        // Calculate and apply power
        double turretOutput = pidf.calculate(currentAngle);
        turretOutput = Range.clip(turretOutput, -maxPower, maxPower);
        turret.setPower(turretOutput);

        double error = targetAngle - currentAngle;

        // --- STANDARD TELEMETRY (Driver Station Phone) ---
        telemetry.addData("--- TURRET TUNING ---", "");
        telemetry.addData("Target", targetAngle);
        telemetry.addData("Actual", "%.2f", currentAngle);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Power", "%.2f", turretOutput);
        double loopTime = loopTimer.milliseconds();

        // --- PANELS TELEMETRY (Custom Dashboard / Graphs) ---
        // These can be used to create live line graphs in the UI
//        panelsTelemetry.debug("Turret/Target", targetAngle);
//        panelsTelemetry.debug("Turret/Actual", currentAngle);
//        panelsTelemetry.debug("Turret/Error", error);
//        panelsTelemetry.debug("Turret/Power", turretOutput);
//        panelsTelemetry.debug("System/Loop MS", loopTimer.milliseconds());
//
//        // Update both systems
//        panelsTelemetry.update(telemetry);

        double remaining = cookedLoopTargetMS - loopTime;
        if (remaining > 0) {
            try {
                Thread.sleep((long) remaining);
            } catch (InterruptedException ignored) {}
        }

        loopTimer.reset();
    }
}