package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Logging.LogRow;
import org.firstinspires.ftc.teamcode.Config.Logging.MatchLogger;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;

public class IntakeWithLogging extends Intake {

    private final MatchLogger logger;

    public IntakeWithLogging(HardwareMap hardwareMap, Telemetry telemetry, MatchLogger logger) {
        super(hardwareMap, telemetry);
        this.logger = logger;
    }

    @Override
    public void update(boolean beamBreak, LimelightCamera.BallOrder targetOrder,
                       DetectedColor s1, DetectedColor s2, DetectedColor s3) {
        super.update(beamBreak, targetOrder, s1, s2, s3);
        LogRow row = logger.getCurrentRow();
        if (row == null) return;
        row.intakeState = getIntakeState().name();
    }
}
