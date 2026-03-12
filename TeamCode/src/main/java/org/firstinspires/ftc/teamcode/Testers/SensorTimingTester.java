package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;

/**
 * SensorTimingTester
 *
 * Measures the execution time (in milliseconds) of every sensor/encoder/IO read.
 * Results are shown on telemetry every loop and logged via RobotLog every LOG_INTERVAL_SEC seconds.
 *
 * How to read the output:
 *   last  = time of the most recent single call (ms)
 *   avg   = running mean across all samples (ms)
 *   min   = fastest single call ever seen (ms)
 *   max   = slowest single call ever seen (ms)
 *   n     = total number of samples collected
 *
 * NOTE on System.nanoTime() vs System.currentTimeMillis():
 *   ElapsedTime uses System.nanoTime() internally — a monotonic hardware counter
 *   that never jumps backward (unlike the wall clock). This is the correct source
 *   for benchmarking because it is unaffected by NTP/clock adjustments and has
 *   nanosecond resolution (practical precision ~100 ns–1 µs on Android).
 *   currentTimeMillis() is wall time, has ~1–15 ms resolution, and can jump.
 *
 */
@Configurable
@TeleOp(name = "Sensor Timing Tester", group = "TEST")
public class SensorTimingTester extends OpMode {

    // --- hardware ---
    private Sensors sensors;
    private GoBildaPinpointDriver pinpoint;

    // motors (encoders)
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx intake1;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;
    private DcMotorEx driveBL;
    private DcMotorEx driveBR;

    // turret servos (NewShooter config — two servos instead of a motor)
    private Servo turret1;
    private Servo turret2;

    // digital channel
    private DigitalChannel beamBreak;

    // servo
    private Servo hoodServo;

    // --- timing infrastructure ---
    private final ElapsedTime callTimer = new ElapsedTime();
    private final ElapsedTime logTimer  = new ElapsedTime();
    private static final double LOG_INTERVAL_SEC = 5.0;

    // ---- SRSHub / Color ----
    private final StatBlock statSensorsUpdate     = new StatBlock("sensors.update()");
    private final StatBlock statSensor1Red        = new StatBlock("colorSensor1.red (field)");
    private final StatBlock statSensor1Green      = new StatBlock("colorSensor1.green (field)");
    private final StatBlock statSensor1Blue       = new StatBlock("colorSensor1.blue (field)");
    private final StatBlock statSensor1Proximity  = new StatBlock("colorSensor1.proximity (field)");
    private final StatBlock statSensor2Red        = new StatBlock("colorSensor2.red (field)");
    private final StatBlock statSensor2Proximity  = new StatBlock("colorSensor2.proximity (field)");
    private final StatBlock statSensor3Red        = new StatBlock("colorSensor3.red (field)");
    private final StatBlock statSensor3Proximity  = new StatBlock("colorSensor3.proximity (field)");
    private final StatBlock statDetectedColor1    = new StatBlock("getDetectedColor(sensor1)");
    private final StatBlock statDetectedColor2    = new StatBlock("getDetectedColor(sensor2)");
    private final StatBlock statDetectedColor3    = new StatBlock("getDetectedColor(sensor3)");
    private final StatBlock statIsHubReady        = new StatBlock("isHubReady()");
    private final StatBlock statIsHubDisconn      = new StatBlock("isHubDisconnected()");
    private final StatBlock statIsCS1Disconn      = new StatBlock("isColorSensor1Disconnected()");

    // ---- GoBilda Pinpoint ----
    private final StatBlock statPinpointUpdate    = new StatBlock("pinpoint.update()");
    private final StatBlock statPinpointPosX      = new StatBlock("pinpoint.getPosX(INCH)");
    private final StatBlock statPinpointPosY      = new StatBlock("pinpoint.getPosY(INCH)");
    private final StatBlock statPinpointHeading   = new StatBlock("pinpoint.getHeading(DEG)");
    private final StatBlock statPinpointVelX      = new StatBlock("pinpoint.getVelX(INCH)");
    private final StatBlock statPinpointVelY      = new StatBlock("pinpoint.getVelY(INCH)");

    // ---- Turret servo position reads ----
    private final StatBlock statTurret1Pos        = new StatBlock("turretServo1.getPosition()");
    private final StatBlock statTurret2Pos        = new StatBlock("turretServo2.getPosition()");

    // ---- Motor encoders: getCurrentPosition() ----
    private final StatBlock statShooter1Pos       = new StatBlock("shooter1.getCurrentPosition()");
    private final StatBlock statShooter2Pos       = new StatBlock("shooter2.getCurrentPosition()");
    private final StatBlock statIntake1Pos        = new StatBlock("intake1.getCurrentPosition()");
    private final StatBlock statDriveFLPos        = new StatBlock("driveFL.getCurrentPosition()");
    private final StatBlock statDriveFRPos        = new StatBlock("driveFR.getCurrentPosition()");
    private final StatBlock statDriveBLPos        = new StatBlock("driveBL.getCurrentPosition()");
    private final StatBlock statDriveBRPos        = new StatBlock("driveBR.getCurrentPosition()");

    // ---- Motor encoders: getVelocity() ----
    private final StatBlock statShooter1Vel       = new StatBlock("shooter1.getVelocity()");
    private final StatBlock statShooter2Vel       = new StatBlock("shooter2.getVelocity()");
    private final StatBlock statIntake1Vel        = new StatBlock("intake1.getVelocity()");
    private final StatBlock statDriveFLVel        = new StatBlock("driveFL.getVelocity()");

    // ---- Digital channel ----
    private final StatBlock statBeamBreak         = new StatBlock("beamBreak.getState()");

    // ---- Servo position reads ----
    private final StatBlock statHoodServoPos      = new StatBlock("hoodServo.getPosition()");

    // -------------------------------------------------------------------------
    @Override
    public void init() {
        telemetry.addLine("Initializing SensorTimingTester...");
        telemetry.update();

        // SRSHub color sensors
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        // Pinpoint odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Flywheel motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "rightShooter");
        intake1  = hardwareMap.get(DcMotorEx.class, "intake1");
        driveFL  = hardwareMap.get(DcMotorEx.class, "fl");
        driveFR  = hardwareMap.get(DcMotorEx.class, "fr");
        driveBL  = hardwareMap.get(DcMotorEx.class, "bl");
        driveBR  = hardwareMap.get(DcMotorEx.class, "br");

        for (DcMotorEx m : new DcMotorEx[]{shooter1, shooter2, intake1, driveFL, driveFR, driveBL, driveBR}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Turret servos (NewShooter config)
        turret1 = hardwareMap.get(Servo.class, "turretServo1");
        turret2 = hardwareMap.get(Servo.class, "turretServo2");

        // Beam break digital channel
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        // Hood servo
        hoodServo = hardwareMap.get(Servo.class, "hood");

        telemetry.addLine("Ready. Run to collect timing data.");
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    @Override
    public void loop() {

        // ================================================================
        // SRSHub / Color
        // ================================================================
        callTimer.reset();
        sensors.update();
        statSensorsUpdate.record(callTimer.milliseconds());

        callTimer.reset(); sensors.getColor1Red();       statSensor1Red.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor1Green();     statSensor1Green.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor1Blue();      statSensor1Blue.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor1Proximity(); statSensor1Proximity.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor2Red();       statSensor2Red.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor2Proximity(); statSensor2Proximity.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor3Red();       statSensor3Red.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getColor3Proximity(); statSensor3Proximity.record(callTimer.milliseconds());

        callTimer.reset(); sensors.getDetectedColor(sensors.colorSensor1); statDetectedColor1.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getDetectedColor(sensors.colorSensor2); statDetectedColor2.record(callTimer.milliseconds());
        callTimer.reset(); sensors.getDetectedColor(sensors.colorSensor3); statDetectedColor3.record(callTimer.milliseconds());

        callTimer.reset(); sensors.isHubReady();                 statIsHubReady.record(callTimer.milliseconds());
        callTimer.reset(); sensors.isHubDisconnected();          statIsHubDisconn.record(callTimer.milliseconds());
        callTimer.reset(); sensors.isColorSensor1Disconnected(); statIsCS1Disconn.record(callTimer.milliseconds());

        // ================================================================
        // GoBilda Pinpoint odometry
        // ================================================================
        callTimer.reset();
        pinpoint.update();
        statPinpointUpdate.record(callTimer.milliseconds());

        callTimer.reset(); pinpoint.getPosX(DistanceUnit.INCH);    statPinpointPosX.record(callTimer.milliseconds());
        callTimer.reset(); pinpoint.getPosY(DistanceUnit.INCH);    statPinpointPosY.record(callTimer.milliseconds());
        callTimer.reset(); pinpoint.getHeading(AngleUnit.DEGREES); statPinpointHeading.record(callTimer.milliseconds());
        callTimer.reset(); pinpoint.getVelX(DistanceUnit.INCH);    statPinpointVelX.record(callTimer.milliseconds());
        callTimer.reset(); pinpoint.getVelY(DistanceUnit.INCH);    statPinpointVelY.record(callTimer.milliseconds());

        // ================================================================
        // Turret servo position reads (SDK-cached, should be ~0 ms)
        // ================================================================
        callTimer.reset(); turret1.getPosition(); statTurret1Pos.record(callTimer.milliseconds());
        callTimer.reset(); turret2.getPosition(); statTurret2Pos.record(callTimer.milliseconds());

        // ================================================================
        // Motor encoder reads — getCurrentPosition() (ticks, integer)
        // These go over I2C to the Control Hub and may cost ~100–500 µs each.
        // With MANUAL bulk caching they become near-free; without it each is a
        // separate I2C transaction.
        // ================================================================
        callTimer.reset(); shooter1.getCurrentPosition(); statShooter1Pos.record(callTimer.milliseconds());
        callTimer.reset(); shooter2.getCurrentPosition(); statShooter2Pos.record(callTimer.milliseconds());
        callTimer.reset(); intake1.getCurrentPosition();  statIntake1Pos.record(callTimer.milliseconds());
        callTimer.reset(); driveFL.getCurrentPosition();  statDriveFLPos.record(callTimer.milliseconds());
        callTimer.reset(); driveFR.getCurrentPosition();  statDriveFRPos.record(callTimer.milliseconds());
        callTimer.reset(); driveBL.getCurrentPosition();  statDriveBLPos.record(callTimer.milliseconds());
        callTimer.reset(); driveBR.getCurrentPosition();  statDriveBRPos.record(callTimer.milliseconds());

        // ================================================================
        // Motor encoder reads — getVelocity() (ticks/sec, double)
        // Same I2C cost as getCurrentPosition() but also applies the SDK's
        // internal velocity calculation.
        // ================================================================
        callTimer.reset(); shooter1.getVelocity(); statShooter1Vel.record(callTimer.milliseconds());
        callTimer.reset(); shooter2.getVelocity(); statShooter2Vel.record(callTimer.milliseconds());
        callTimer.reset(); intake1.getVelocity();  statIntake1Vel.record(callTimer.milliseconds());
        callTimer.reset(); driveFL.getVelocity();  statDriveFLVel.record(callTimer.milliseconds());

        // ================================================================
        // Digital channel (beam break) — single digital GPIO read
        // ================================================================
        callTimer.reset();
        beamBreak.getState();
        statBeamBreak.record(callTimer.milliseconds());

        // ================================================================
        // Servo position read — cached locally in the SDK, should be ~0 ms
        // ================================================================
        callTimer.reset();
        hoodServo.getPosition();
        statHoodServoPos.record(callTimer.milliseconds());

        // ================================================================
        // Telemetry display
        // ================================================================
        telemetry.addLine("=== TIMING (ms) ===  [last | avg | min | max | n]");

        telemetry.addLine("--- SRSHub / Color ---");
        addRow(statSensorsUpdate);
        addRow(statSensor1Red);     addRow(statSensor1Green); addRow(statSensor1Blue);
        addRow(statSensor1Proximity);
        addRow(statSensor2Red);     addRow(statSensor2Proximity);
        addRow(statSensor3Red);     addRow(statSensor3Proximity);
        addRow(statDetectedColor1); addRow(statDetectedColor2); addRow(statDetectedColor3);
        addRow(statIsHubReady);     addRow(statIsHubDisconn);   addRow(statIsCS1Disconn);

        telemetry.addLine("--- GoBilda Pinpoint ---");
        addRow(statPinpointUpdate);
        addRow(statPinpointPosX);   addRow(statPinpointPosY);
        addRow(statPinpointHeading);
        addRow(statPinpointVelX);   addRow(statPinpointVelY);

        telemetry.addLine("--- Turret Servos ---");
        addRow(statTurret1Pos);  addRow(statTurret2Pos);

        telemetry.addLine("--- Motor Encoders: getCurrentPosition() ---");
        addRow(statShooter1Pos); addRow(statShooter2Pos);
        addRow(statIntake1Pos);
        addRow(statDriveFLPos);  addRow(statDriveFRPos);
        addRow(statDriveBLPos);  addRow(statDriveBRPos);

        telemetry.addLine("--- Motor Encoders: getVelocity() ---");
        addRow(statShooter1Vel); addRow(statShooter2Vel);
        addRow(statIntake1Vel);  addRow(statDriveFLVel);

        telemetry.addLine("--- Digital / Servo ---");
        addRow(statBeamBreak);
        addRow(statHoodServoPos);

        telemetry.addData("total samples", statSensorsUpdate.n);
        telemetry.update();

        // ================================================================
        // Periodic RobotLog dump (searchable tag: "SensorTiming")
        // ================================================================
        if (logTimer.seconds() >= LOG_INTERVAL_SEC) {
            logTimer.reset();
            dumpToRobotLog();
        }
    }

    // -------------------------------------------------------------------------
    private void addRow(StatBlock s) {
        telemetry.addData(s.name, "%.3f | %.3f | %.3f | %.3f | %d",
                s.last, s.avg(), s.min, s.max, s.n);
    }

    private void dumpToRobotLog() {
        long n = statSensorsUpdate.n;
        RobotLog.ii("SensorTiming", "====== SENSOR TIMING DUMP (%d samples) ======", n);
        RobotLog.ii("SensorTiming", "%-45s  last(ms)  avg(ms)  min(ms)  max(ms)", "Method");

        RobotLog.ii("SensorTiming", "--- SRSHub / Color ---");
        logStat(statSensorsUpdate);
        logStat(statSensor1Red);     logStat(statSensor1Green); logStat(statSensor1Blue);
        logStat(statSensor1Proximity);
        logStat(statSensor2Red);     logStat(statSensor2Proximity);
        logStat(statSensor3Red);     logStat(statSensor3Proximity);
        logStat(statDetectedColor1); logStat(statDetectedColor2); logStat(statDetectedColor3);
        logStat(statIsHubReady);     logStat(statIsHubDisconn);   logStat(statIsCS1Disconn);

        RobotLog.ii("SensorTiming", "--- GoBilda Pinpoint ---");
        logStat(statPinpointUpdate);
        logStat(statPinpointPosX);   logStat(statPinpointPosY);
        logStat(statPinpointHeading);
        logStat(statPinpointVelX);   logStat(statPinpointVelY);

        RobotLog.ii("SensorTiming", "--- Turret Servos ---");
        logStat(statTurret1Pos);  logStat(statTurret2Pos);

        RobotLog.ii("SensorTiming", "--- Motor Encoders: getCurrentPosition() ---");
        logStat(statShooter1Pos); logStat(statShooter2Pos);
        logStat(statIntake1Pos);
        logStat(statDriveFLPos);  logStat(statDriveFRPos);
        logStat(statDriveBLPos);  logStat(statDriveBRPos);

        RobotLog.ii("SensorTiming", "--- Motor Encoders: getVelocity() ---");
        logStat(statShooter1Vel); logStat(statShooter2Vel);
        logStat(statIntake1Vel);  logStat(statDriveFLVel);

        RobotLog.ii("SensorTiming", "--- Digital / Servo ---");
        logStat(statBeamBreak);
        logStat(statHoodServoPos);

        RobotLog.ii("SensorTiming", "=============================================");
    }

    private void logStat(StatBlock s) {
        RobotLog.ii("SensorTiming", "%-45s  %8.4f  %7.4f  %7.4f  %7.4f",
                s.name, s.last, s.avg(), s.min, s.max);
    }

    // =========================================================================
    /** Lightweight running statistics — no allocations after construction. */
    private static class StatBlock {
        final String name;
        double last = 0;
        double min  = Double.MAX_VALUE;
        double max  = Double.MIN_VALUE;
        double sum  = 0;
        long   n    = 0;

        StatBlock(String name) { this.name = name; }

        void record(double ms) {
            last = ms;
            if (ms < min) min = ms;
            if (ms > max) max = ms;
            sum += ms;
            n++;
        }

        double avg() { return n == 0 ? 0 : sum / n; }
    }
}
