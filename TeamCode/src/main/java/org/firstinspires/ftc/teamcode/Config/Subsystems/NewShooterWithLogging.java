package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Logging.LogRow;
import org.firstinspires.ftc.teamcode.Config.Logging.MatchLogger;

public class NewShooterWithLogging extends NewShooter {

    private final MatchLogger logger;

    public NewShooterWithLogging(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto, MatchLogger logger) {
        super(hardwareMap, telemetry, isAuto);
        this.logger = logger;
    }

    @Override
    public void update(double currentTurretAngle0_360) {
        super.update(currentTurretAngle0_360);
        LogRow row = logger.getCurrentRow();
        if (row == null) return;

        row.flywheelVeloActual  = getFlywheelVelo();
        row.flywheelVeloTarget  = getTargetFLywheelVelo();
        row.flywheelVeloReached = flywheelVeloReached;
        row.turretActualDeg     = getCurrentTurretPosition();
        row.turretTargetDeg     = getTargetTurretPosition();
        row.turretReached       = turretReached;
        row.hoodTargetDeg       = getCurrentRequiredHoodAngle();
        row.hoodReached         = hoodReached;
        row.beamBroken          = isBeamBroken();
    }
}
