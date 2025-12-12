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

import java.util.List;




//@Disabled
@Configurable
@TeleOp(name = "FlywheelPIDFTuner", group = "TEST")
public class FlywheelPIDFTuner extends OpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.007;       // proportional gain
    public static double kI = 0.000;      // integral gain
    public static double kD = 0.000;      // derivative gain
    public static double kF = 0.00042;      // feedforward ≈ 1 / maxTicksPerSec  old is 0.00042
    public static double targetVelocity = 1633; // desired speed (ticks/sec)
    public static double maxPower = 1.0;          // safety clamp
    public static final int TICKS_PER_REV = 537;
    private static final String HUB_NAME = "SRSHub";
    public static boolean intakeOn = false;
    public static double cookedLoopTargetMS = 100;

    // ===== Hardware =====
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    private PIDFController pidf;
    private Sensors sensors;

    private ElapsedTime loopTimer = new ElapsedTime();
    // Removed lastTicks and lastTime since they're no longer needed for manual calculation

    @Override
    public void init() {
        //  Enable bulk reads for faster loops
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        shooter1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "motor2");
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            // The Sensors class handles finding and configuring the SRSHub itself
            sensors.init(hardwareMap, HUB_NAME);
            telemetry.addData("Status", "Initialization Successful!");
        } catch (Exception e) {
            telemetry.addData("Status", "FATAL ERROR during initialization!");
            telemetry.addData("Error", e.getMessage());

        }
        telemetry.update();


        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        double rPM = sensors.getFlywheelVelo();

        // --- Update gains and setpoint ---
        pidf.setPIDF(kP, kI, kD, kF);
        //pidf.setSetPoint(targetVelocity);

        // --- Compute output and send to motor ---
        // 'rPM' (the measured velocity) is passed to the custom PIDF controller.
        double output = pidf.calculate(rPM, targetVelocity);
        output = Range.clip(output, 0, maxPower);
        shooter1.setPower(output);
        shooter2.setPower(output);


        //YALL SHOULD USE THIS INSTEAD
//        shooter1.setVelocity(output);
//        shooter2.setVelocity(output);




        // --- Telemetry ---
        double loopTime = loopTimer.milliseconds();
        telemetry.addData("Target Vel (ticks/s)", targetVelocity);
        telemetry.addData("Measured Vel (ticks/s)", rPM);
        telemetry.addData("Output Power", output);
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