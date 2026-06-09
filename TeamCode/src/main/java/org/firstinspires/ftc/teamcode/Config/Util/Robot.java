package org.firstinspires.ftc.teamcode.Config.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

public class Robot {

    public MecanumDrive drive;
    public Intake intake;
    public NewShooter shooter;
    public LimelightCamera limelight;
    public Sensors sensors;

    // Loop time tracking
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopTimeMs = 0;
    private double minLoopTimeMs = Double.MAX_VALUE;
    private double maxLoopTimeMs = 0;
    private double totalLoopTimeMs = 0;
    private long loopCount = 0;

    private PrintWriter logWriter;

    public void init(HardwareMap hardwareMap) {
        drive = new MecanumDrive();
        drive.init(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new NewShooter(hardwareMap, telemetry, false);
        limelight = new LimelightCamera(hardwareMap);

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        try {
            logWriter = new PrintWriter(new BufferedWriter(
                    new FileWriter("/sdcard/FIRST/loop_log_" + timestamp + ".txt")));
            logWriter.println("=== Loop Log: " + timestamp + " ===");
            logWriter.flush();
        } catch (IOException e) {
            logWriter = null;
        }
    }

    /** Call at the very start of each OpMode loop iteration. */
    public void startLoop() {
        loopTimer.reset();
    }

    /**
     * Call at the very end of each OpMode loop iteration.
     * Updates stats, logs to RobotLog, and pushes telemetry lines.
     */
    public void endLoop(Telemetry t) {
        loopTimeMs = loopTimer.milliseconds();
        loopCount++;
        totalLoopTimeMs += loopTimeMs;
        if (loopTimeMs < minLoopTimeMs) minLoopTimeMs = loopTimeMs;
        if (loopTimeMs > maxLoopTimeMs) maxLoopTimeMs = loopTimeMs;

        double avgLoopTimeMs = totalLoopTimeMs / loopCount;

        if (logWriter != null) {
            logWriter.printf("Loop #%d: %.2fms (min=%.2f max=%.2f avg=%.2f)%n",
                    loopCount, loopTimeMs, minLoopTimeMs, maxLoopTimeMs, avgLoopTimeMs);
        }

        t.addData("[Loop] Time (ms)", "%.2f", loopTimeMs);
        t.addData("[Loop] Min/Max/Avg (ms)", "%.2f / %.2f / %.2f",
                minLoopTimeMs, maxLoopTimeMs, avgLoopTimeMs);
        t.addData("[Loop] Count", loopCount);
    }

    /** Returns the most recent loop time in milliseconds. */
    public double getLoopTimeMs() { return loopTimeMs; }

    /** Resets all loop time statistics. */
    public void resetLoopStats() {
        loopTimeMs = 0;
        minLoopTimeMs = Double.MAX_VALUE;
        maxLoopTimeMs = 0;
        totalLoopTimeMs = 0;
        loopCount = 0;
    }

    /** Flush and close the log file. Call from OpMode stop(). */
    public void stop() {
        if (logWriter != null) {
            logWriter.printf("=== Session end — total loops: %d ===%n", loopCount);
            logWriter.flush();
            logWriter.close();
            logWriter = null;
        }
    }
}
