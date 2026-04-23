package org.firstinspires.ftc.teamcode.Config.Logging;

public class LogRow {

    // Metadata
    public int loopCount   = -1;
    public long timestampNs = -1;  // nanoseconds elapsed since match start

    // Robot pose & motion (reads)
    public double robotX           = Double.NaN;
    public double robotY           = Double.NaN;
    public double robotHeadingDeg  = Double.NaN;
    public double robotVelocity    = Double.NaN;
    public double pathTValue       = Double.NaN;

    // Autonomous state
    public int pathState              = -1;
    public int ballsShotInState       = -1;
    public boolean goForLaunch;
    public boolean intakeStoppedForShooting;
    public boolean beamWasCleared;

    // Flywheel
    public double flywheelVeloActual   = Double.NaN;  // read
    public double flywheelVeloTarget   = Double.NaN;  // write (commanded)
    public boolean flywheelVeloReached;               // ±300 ticks tolerance
    public boolean veloReachedTight;                  // ±40 ticks tolerance (OpMode gate)

    // Turret
    public double turretActualDeg  = Double.NaN;  // read
    public double turretTargetDeg  = Double.NaN;  // write (commanded)
    public boolean turretReached;

    // Hood
    public double hoodTargetDeg = Double.NaN;  // write (commanded)
    public boolean hoodReached;

    // Beam break (read)
    public boolean beamBroken;

    // Intake
    public String intakeState = "N/A";

    public static String csvHeader() {
        return "loopCount,timestampMs," +
               "robotX,robotY,robotHeadingDeg,robotVelocity,pathTValue," +
               "pathState,ballsShotInState,goForLaunch,intakeStoppedForShooting,beamWasCleared," +
               "flywheelVeloActual,flywheelVeloTarget,flywheelVeloReached,veloReachedTight," +
               "turretActualDeg,turretTargetDeg,turretReached," +
               "hoodTargetDeg,hoodReached," +
               "beamBroken," +
               "intakeState";
    }

    public String toCsvLine() {
        return loopCount + "," +
               (timestampNs / 1_000_000) + "," +
               robotX + "," +
               robotY + "," +
               robotHeadingDeg + "," +
               robotVelocity + "," +
               pathTValue + "," +
               pathState + "," +
               ballsShotInState + "," +
               goForLaunch + "," +
               intakeStoppedForShooting + "," +
               beamWasCleared + "," +
               flywheelVeloActual + "," +
               flywheelVeloTarget + "," +
               flywheelVeloReached + "," +
               veloReachedTight + "," +
               turretActualDeg + "," +
               turretTargetDeg + "," +
               turretReached + "," +
               hoodTargetDeg + "," +
               hoodReached + "," +
               beamBroken + "," +
               intakeState;
    }
}
