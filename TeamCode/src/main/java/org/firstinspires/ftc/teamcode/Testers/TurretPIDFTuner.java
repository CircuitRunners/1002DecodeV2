package org.firstinspires.ftc.teamcode.Testers;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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




//@Disabled
@Configurable
@TeleOp(name = "TurretPIDFTuner", group = "TEST")
public class TurretPIDFTuner extends OpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.007;       // proportional gain
    public static double kI = 0.000;      // integral gain
    public static double kD = 0.000;      // derivative gain
    public static double kF = 0.00042;      // feedforward ≈ 1 / maxTicksPerSec  old is 0.00042
    public static double targetAngle = 180;
    private ElapsedTime settleTimer = new ElapsedTime();
    private boolean isMoving = false;
    private static double tolerance = 5.0; // deg

    public static double maxPower = 1.0;          // safety clamp
    private Sensors sensors;
    private static final String HUB_NAME = "SRSHub";
    public static boolean intakeOn = false;
    public static double cookedLoopTargetMS = 100;

    // ===== Hardware =====
    private Shooter turret;
//    private PIDFController pidf;

    private ElapsedTime loopTimer = new ElapsedTime();
    // Removed lastTicks and lastTime since they're no longer needed for manual calculation

    @Override
    public void init() {
        //  Enable bulk reads for faster loops
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        turret = new Shooter(hardwareMap, telemetry);
        sensors = new Sensors();
        sensors.init(hardwareMap, HUB_NAME);

//        pidf = new PIDFController(kP, kI, kD, kF);
//        pidf.setSetPoint(targetAngle);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        double currentAngle = sensors.getTurretPosition();

        turret.turretPIDF.setPIDF(kP, kI, kD, kF);

        turret.setTurretTargetPosition(targetAngle);
        turret.update(0, currentAngle, 0); //flywheel and hood irrelevant

        double error = Math.abs(targetAngle - currentAngle);

        if (!isMoving && error > tolerance) {
            isMoving = true;
            settleTimer.reset();
        }

        if (isMoving && error <= tolerance) {
            isMoving = false;
            telemetry.addData("Settled Time (ms)", settleTimer.milliseconds());
            telemetry.update();
        }




        // --- Telemetry ---
        double loopTime = loopTimer.milliseconds();
        telemetry.addData("Current Angle (Deg): ", currentAngle);
        telemetry.addData("Target Angle (Deg)", targetAngle);
        telemetry.addData("Turret Error", Math.abs(targetAngle - currentAngle));
        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Cooked Delay (ms)", cookedLoopTargetMS);
        telemetry.update();

        // --- Cooked delay to simulate TeleOp lag (~100 ms total loop) ---
        double remaining = cookedLoopTargetMS - loopTime;
        if (remaining > 0) {
            try {
                Thread.sleep((long) remaining);
            } catch (InterruptedException ignored) {}
        }

        loopTimer.reset();
    }
}