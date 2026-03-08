package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class NewShooter {

    //NOTE FOR ALL - DEGREES INCREASE COUNTERCLOCKWISE LIKE UNIT CIRCLE
    private Telemetry telemetry;

    private static final double transferTimeOffsetSec = 0.0; //TUNE

    public static double turretEndPosAuto = 0;

    // PIDF Coefficients
    public static double[] flywheelCoefficients = {1.1, 0.000001, 0.00465, 0.000071};
    public static double[] turretCoefficientsTeleop = {0.06, 0.00, 0.00225, 0.0024125};
    public static double[] turretCoefficientsAuto = {0.06, 0.00, 0.00225, 0.0024125};

    // Target States
    private static double targetFlywheelVelocity = 0;   // Ticks/Sec
    private static double targetTurretPosition = 0;
    private static double targetHoodAngle = 45;          // Degrees (0-90)

    private static double maxTurretPower = 0.8;

    private static final double GRAVITY = 386.088; // in/s^2

    public static double turretDeadband = 0;

    // Heights (inches)
    private static final double LAUNCH_HEIGHT_IN = 16;
    private static final double GOAL_HEIGHT_IN = 38.75;
    private static final double GOAL_HEIGHT_SAFETY_OFFSET_IN = 2.0;

    final double GOAL_HEIGHT = 38.75;
    final double ANGLE_BIAS_DEG = -18.35;
    final double G_INCHES = 386.088;
    final double VECTOR_TUNE = 2.49;

    private static final double EFFECTIVE_GOAL_HEIGHT_IN =
            GOAL_HEIGHT_IN + GOAL_HEIGHT_SAFETY_OFFSET_IN;

    private static final double MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC = 400000;

    private static double MIN_LAUNCH_ANGLE_DEG = 5.0;
    private static double MAX_LAUNCH_ANGLE_DEG = 45.0;

    private static final double ANGLE_SEARCH_STEP_DEG = 2.5;

    // Status flags
    public static boolean flywheelVeloReached = false;
    public static boolean turretReached = false;
    public static boolean hoodReached = false;

    public double turret1MinRange = 0.0; //old was -720, 720
    public double turret1MaxRange = 1.0;
    public double turret2MinRange = 0.0; //old was -720, 720
    public double turret2MaxRange = 1.0;

    public boolean hoodCalibrationRequired = false;

    // Hardware
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private Servo turret1;
    private Servo turret2;
    private Servo hoodServo;

    private PIDFController flywheelPIDF;

    private DigitalChannel shooterBeamBreak;

    // Hood Coefficients
    public static double h_a = -(2.52835e-7), h_b = 0.00012437, h_c = -0.0230779, h_d = 1.9269, h_e = -17.60642;

    /**
     * Flywheel velocity lookup table
     *
     * distanceInches , flywheelTicksPerSec
     *
     * MUST be sorted by distance (ascending)
     */
    private static final double[][] VELO_LUT = {
            { 20.0,  1065},
            { 30.0, 1125 },
            { 40.0, 1175 },
            { 50.0, 1175 },
            { 60.0, 1219 },
            { 70.0, 1253 },
            { 80.0, 1297 },
            { 90.0, 1335},
            { 100.0, 1427 },
            { 110.0, 1542 },
            { 120.0, 1580 },
            { 130.0, 1655 },
            { 140.0, 1684 },
            { 145.0, 1688 }
    };

    /**
     * Shot time lookup table
     *
     * distanceInches , shotTime (seconds)
     *
     * MUST be sorted by distance (ascending)
     */
    private static final double[][] SHOT_TIME_LUT = {
            { 20.0,  0.42},
            { 30.0, 0.45},
            { 40.0, 0.49},
            { 50.0, 0.53},
            { 60.0, 0.57},
            { 70.0, 0.61},
            { 80.0, 0.64},
            { 90.0, 0.70},
            { 100.0, 0.73},
            { 110.0, 0.78},
            { 120.0, 0.82},
            { 130.0, 0.87},
            { 140.0, 0.9},
            { 145.0, 0.92}
    };

    public NewShooter(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto) {
        this.telemetry = telemetry;

        shooter1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret1 = hardwareMap.get(Servo.class, "turretServo1");
        turret2 = hardwareMap.get(Servo.class, "turretServo2");

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterBeamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        shooterBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        flywheelPIDF = new PIDFController(
                flywheelCoefficients[0], flywheelCoefficients[1],
                flywheelCoefficients[2], flywheelCoefficients[3]
        );

        flywheelPIDF.setSetPoint(0);
        setHoodTargetAngle(45);
    }

    /**
     * Normalizes an angle from [-180, 180] to [0, 360] (Robot Heading).
     */
    private static double normalizeRobotHeading0_360(double headingDeg) {
        double normalized = headingDeg % 360;
        normalized *= -1;
        if (normalized < 0) normalized += 360;
        return normalized;
    }

    public static double calculateAutoAlignYaw(double robotXInches, double robotYInches,
                                               double targetXInches, double targetYInches, boolean isRed) {
        double deltaY = targetYInches - robotYInches;
        double deltaX = targetXInches - robotXInches;

        // Standard atan2(y, x) for East = 0, North = 90
        double targetFieldYawRad = Math.atan2(-deltaY, deltaX);
        double targetFieldYawDeg = Math.toDegrees(targetFieldYawRad);
        if (targetFieldYawDeg < 0) {
            targetFieldYawDeg += 360;
        }

        return Math.round(targetFieldYawDeg);
    }

    // ------------------------------------
    // ##  Turret Control
    // ------------------------------------
    public enum TurretMode {
        ROBOT_CENTRIC, FIELD_CENTRIC, AUTO_ALIGN
    }

    public void setTurretTarget(double inputFieldDeg, TurretMode mode, double robotFieldYawDeg, double mannualTurretAdjust) {
        double absoluteTarget = 0;
        robotFieldYawDeg = normalizeRobotHeading0_360(robotFieldYawDeg);

        switch (mode) {
            case FIELD_CENTRIC:
                // inputFieldDeg is your constant (e.g., 180 for West)
                // Subtract robot heading to find robot-relative angle
                absoluteTarget = Range.clip((((inputFieldDeg - robotFieldYawDeg + 360) % 360)), 0, 360);
                break;

            case AUTO_ALIGN:
                // Input is already the field yaw calculated from (dx, dy)
                absoluteTarget = Range.clip((((inputFieldDeg - robotFieldYawDeg + 360) % 360)), 0, 360);
                break;

            case ROBOT_CENTRIC:
                // Directly setting the turret 0-360 relative to the front of the robot
                absoluteTarget = Range.clip(inputFieldDeg, -360, 360);
                break;
        }

        setTurretTargetPosition(absoluteTarget + mannualTurretAdjust);
    }

    public void setTurretTargetPosition(double positionDeg) {
        // Map 0->360 into -180->180
        // If input is 0..180, it stays 0..180
        // If input is 181..360, it becomes -179..0
        double physicalTarget;
        if (positionDeg <= 180) {
            physicalTarget = positionDeg;
        } else {
            physicalTarget = positionDeg - 360;
        }

        // --- WIRING DEAD ZONE HANDLING ---
        // If target is in the 315-360 deadzone, pick the closest safe limit
        if (Math.abs(physicalTarget) >= 178) {
            if (physicalTarget < 0) { // Closer to 315
                physicalTarget = -178;
            }
        }

        // Clip to stay away from the physical hardstops
        targetTurretPosition = Range.clip(physicalTarget, -178, 180);
    }

    // ------------------------------------
    // ##  Flywheel Control
    // ------------------------------------

    public void setTargetVelocityTicks(double targetTicksPerSec) {
        targetFlywheelVelocity = Range.clip(targetTicksPerSec, 0, MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC);
    }

    // ------------------------------------
    // ##  Update Loop
    // ------------------------------------

    public void update(double currentTurretAngle0_360) {
        double shooter1Velocity = shooter1.getVelocity();
        double shooter2Velocity = shooter2.getVelocity();

        double averageVelo = (shooter1Velocity + shooter2Velocity) / 2;

        if (averageVelo >= targetFlywheelVelocity - 300 || averageVelo <= targetFlywheelVelocity + 300) {
            flywheelVeloReached = true;
        } else {
            flywheelVeloReached = false;
        }

        if (currentTurretAngle0_360 >= targetTurretPosition - 0.7 || currentTurretAngle0_360 <= targetTurretPosition + 0.7) {
            turretReached = true;
        } else {
            turretReached = false;
        }

        hoodCalibrationRequired = false;

        flywheelPIDF.setPIDF(flywheelCoefficients[0], flywheelCoefficients[1], flywheelCoefficients[2], flywheelCoefficients[3]);

        double flywheelOutput = flywheelPIDF.calculate(averageVelo, targetFlywheelVelocity);
        flywheelOutput = Range.clip(flywheelOutput, 0, 1);

        if (Math.abs(averageVelo) <= 85 && Math.abs(targetFlywheelVelocity - averageVelo) <= 10) {
            shooter1.setPower(0);
            shooter2.setPower(0);
        } else {
            shooter1.setPower(flywheelOutput);
            shooter2.setPower(flywheelOutput);
        }

        updateTurretPosition(currentTurretAngle0_360);

        // Hood Control (No PIDF)
        hoodServo.setPosition(hoodAngleToServoPos(targetHoodAngle));
        hoodReached = Math.abs(getHoodServoPositionInDegrees(hoodServo.getPosition()) - targetHoodAngle) <= 1.5;
    }

    // ------------------------------------
    // ##  Helpers
    // ------------------------------------

    /**
     * Returns interpolated flywheel velocity (ticks/sec)
     * for a given distance in inches.
     */
    private static double getFlywheelVeloFromDistanceLUT(double distanceInches) {
        if (distanceInches <= VELO_LUT[0][0]) {
            return VELO_LUT[0][1];
        }
        if (distanceInches >= VELO_LUT[VELO_LUT.length - 1][0]) {
            return VELO_LUT[VELO_LUT.length - 1][1];
        }
        for (int i = 0; i < VELO_LUT.length - 1; i++) {
            double d0 = VELO_LUT[i][0];
            double v0 = VELO_LUT[i][1];
            double d1 = VELO_LUT[i + 1][0];
            double v1 = VELO_LUT[i + 1][1];
            if (distanceInches >= d0 && distanceInches <= d1) {
                double t = (distanceInches - d0) / (d1 - d0);
                return v0 + t * (v1 - v0);
            }
        }
        return VELO_LUT[0][1];
    }

    /**
     * Returns interpolated shot time (seconds)
     * for a given distance in inches.
     */
    public static double getShottimeFromDistanceLUT(double distanceInches) {
        if (distanceInches <= SHOT_TIME_LUT[0][0]) {
            return SHOT_TIME_LUT[0][1];
        }
        if (distanceInches >= SHOT_TIME_LUT[SHOT_TIME_LUT.length - 1][0]) {
            return SHOT_TIME_LUT[SHOT_TIME_LUT.length - 1][1];
        }
        for (int i = 0; i < SHOT_TIME_LUT.length - 1; i++) {
            double d0 = SHOT_TIME_LUT[i][0];
            double v0 = SHOT_TIME_LUT[i][1];
            double d1 = SHOT_TIME_LUT[i + 1][0];
            double v1 = SHOT_TIME_LUT[i + 1][1];
            if (distanceInches >= d0 && distanceInches <= d1) {
                double t = (distanceInches - d0) / (d1 - d0);
                return v0 + t * (v1 - v0);
            }
        }
        return SHOT_TIME_LUT[0][1];
    }

    /**
     * Maps a desired hood angle to the required servo position [0.0, 1.0] using linear scaling.
     */
    private double hoodAngleToServoPos(double angle) {
        return Range.scale(
                angle,
                MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG,
                0.2, 0.025
        );
    }

    /**
     * Maps the current servo position [0.0, 1.0] back to the current hood angle in degrees.
     */
    private double getHoodServoPositionInDegrees(double currentServoPos) {
        return Range.scale(
                currentServoPos,
                0.2, 0.025,
                MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG
        );
    }

    public void setHoodTargetAngle(double angle) {
        targetHoodAngle = angle;
    }

    public void setFlywheelPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void stopFlywheel() {
        setFlywheelPower(0);
        setTargetVelocityTicks(0);
    }

    public double getCurrentRequiredFlywheelTicks() {
        return targetFlywheelVelocity;
    }

    public double getCurrentRequiredHoodAngle() {
        return targetHoodAngle;
    }

    public boolean isBeamBroken() {
        return !shooterBeamBreak.getState();
    }

    static final double TICKS_PER_REV = 4096.0;
    static final double WHEEL_RADIUS_IN = 1.4173; // 72mm / 2 -> inches
    static final double G = 386.0; // in/s^2
    public static double k = 0.4;

    private double calcHoodAngle(double distance) {
        return (h_a * Math.pow(distance, 4)) + (h_b * Math.pow(distance, 3)) +
                (h_c * Math.pow(distance, 2)) + (h_d * distance) + h_e;
    }

    public void setTargetsByDistance(double robotX, double robotY, double goalX, double goalY, double robotAngle, boolean autoAlign, double hoodMannualAdjustment, boolean isRed, double turretAdjustment) {
        double x = Math.hypot(goalX - robotX, goalY - robotY);

        setTargetVelocityTicks(getFlywheelVeloFromDistanceLUT(x));
        setHoodTargetAngle(Range.clip(calcHoodAngle(x) + hoodMannualAdjustment, 0, 45));
        if (autoAlign) {
            double requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY, isRed);
            setTurretTarget(requiredFieldYaw, TurretMode.AUTO_ALIGN, robotAngle, turretAdjustment);
        }
    }

    public void setTargetsByDistanceAdjustable(double robotX, double robotY, double goalX, double goalY, double robotAngle, boolean autoAlign, double flywheelMannualAdjustment, double hoodMannualAdjustment, boolean isRed, double turretManualAdjustment) {
        double x = Math.hypot(goalX - robotX, goalY - robotY);

        setTargetVelocityTicks(getFlywheelVeloFromDistanceLUT(x) + flywheelMannualAdjustment);
        setHoodTargetAngle(Range.clip(calcHoodAngle(x) + hoodMannualAdjustment, 0, 45));
        if (autoAlign) {
            double adjustedGoalY = isRed ? goalY : goalY + 3;
            double requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, adjustedGoalY, isRed);
            setTurretTarget(requiredFieldYaw, TurretMode.AUTO_ALIGN, robotAngle, turretManualAdjustment);
        }
    }

    public double getCurrentTurretAngle() {
        double turret1CurrentTurretAngle = Range.scale(turret1.getPosition(), turret1MinRange, turret1MaxRange, -180, 180);
        double turret2CurrentTurretAngle = Range.scale(turret2.getPosition(), turret2MinRange, turret2MaxRange, -180, 180);
        return (turret1CurrentTurretAngle + turret2CurrentTurretAngle) / 2;
    }

    public void updateTurretPosition(double currentAngle) {
        if (Math.abs(targetTurretPosition - currentAngle) <= turretDeadband) {
            return;
        } else if (targetTurretPosition == 0 && ((targetTurretPosition - currentAngle) <= 2)) {
            return;
        } else {
            double turret1TargetPosition = Range.scale(targetTurretPosition, -180, 180, turret1MinRange, turret1MaxRange);
            double turret2TargetPosition = Range.scale(targetTurretPosition, -180, 180, turret2MinRange, turret2MaxRange);
            turret1.setPosition(turret1TargetPosition);
            turret2.setPosition(turret2TargetPosition);
        }
    }

    public double getTurretTargetPosition() {
        return targetTurretPosition;
    }

    public double[] computeVelocityCompensatedPositionFirestorm(
            double targetX,
            double targetY,
            double robotX,
            double robotY,
            double velX,
            double velY
    ) {
        double correctedX = targetX;
        double correctedY = targetY;

        double distance = Math.hypot(targetX - robotX, targetY - robotY);
        double timeOfFlight = getShottimeFromDistanceLUT(distance);

        for (int i = 0; i < 20; i++) {
            correctedX = targetX - (velX * timeOfFlight);
            correctedY = targetY - (velY * timeOfFlight);

            distance = Math.hypot(correctedX - robotX, correctedY - robotY);
            timeOfFlight = getShottimeFromDistanceLUT(distance);
        }

        return new double[] { correctedX, correctedY };
    }

    public void rezeroTurretPosition() {
        setTurretTarget(0, TurretMode.ROBOT_CENTRIC, getCurrentTurretAngle(), 0);
    }

    public double getTargetFLywheelVelo() {
        return targetFlywheelVelocity;
    }

    public double calcFlywheelSpeedInches(double flywheelSpeedTicks) {
        return flywheelSpeedTicks * ((72 * Math.PI) / (4096 * 25.4));
    }

    public double calcFlywheelSpeedTicks(double flywheelSpeedInches) {
        return flywheelSpeedInches / ((72 * Math.PI) / (4096 * 25.4));
    }

    public double getFlywheelVelo() {
        return (shooter1.getVelocity() + shooter2.getVelocity()) / 2;
    }
}
