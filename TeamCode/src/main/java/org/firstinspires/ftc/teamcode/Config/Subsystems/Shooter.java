package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter {

    //NOTE FOR ALL - DEGREES INCREASE COUNTERCLOCKWISE LIKE UNIT CIRCLE
    private Telemetry telemetry;

    // PIDF Coefficients
    private static final double[] flywheelCoefficients = {0.002, 0, 0.0001, 0.000423};
    private static final double[] turretCoefficients = {0.01, 0, 0.0001, 0.005};

    // Target States
    private static double targetFlywheelVelocity = 0;   // Ticks/Sec
    private static double targetTurretPosition = 180;     // Degrees (0-360)
    private static double targetHoodAngle = 45;
    private static final double HOOD_MIN_ANGLE_DEG = 0;
    private static final double HOOD_MAX_ANGLE_DEG = 90;

    private static final double HOOD_MIN_POS = 0.23;   // measured
    private static final double HOOD_MAX_POS = 0.78;   // measured
    // Degrees (0-90)
    private static double maxTurretPower = 0.8;

    // Hardware Constants
    //public static final int TICKS_PER_REV = 537; // Example value for a common motor

    // --- PHYSICS CONSTANTS ---
    private static final double GRAVITY_INCHES_PER_SEC_SQ = 386.1; // g in in/s^2
    private static final double GOAL_HEIGHT_INCHES = 38.75;       // Example goal height
    private static final double GOAL_HEIGHT_SAFETY_OFFSET_INCHES = 1.5;
    private static final double LAUNCH_HEIGHT_INCHES = 12.0;

    // --- SHOT CONSTRAINTS (For Iterative Search) ---
    private static final double MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC = 2333.33;

    // ⭐ MODIFIED: These are now the dynamic calibration limits
    private static double MIN_LAUNCH_ANGLE_DEG = 15.0;
    private static double MAX_LAUNCH_ANGLE_DEG = 60.0;

    private static final double ANGLE_SEARCH_STEP_DEG = 0.5;

    // Status flags
    public boolean flywheelVeloReached;
    public boolean turretReached;
    public boolean hoodReached;

    // ⭐ MODIFIED: Public flag for external classes to reference
    public boolean hoodCalibrationRequired = false;

    // Hardware
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx turret;
    private Servo hoodServo;

    private PIDFController flywheelPIDF;
    private PIDFController turretPIDF;

    /**
     * Data structure for the calculated optimal shot parameters.
     * Contains only ballistics data (velocity, angle, time).
     * NOTE: Velocity is in Ticks/Sec.
     */
    public static class OptimalShot {
        public double requiredFlywheelTicks;    // Ticks/Sec
        public double requiredHoodAngle;        // degrees (0-90)
        public double timeOfFlight;             // seconds

        public OptimalShot(double ticks, double reqAngle, double tof) {
            this.requiredFlywheelTicks = ticks;
            this.requiredHoodAngle = reqAngle;
            this.timeOfFlight = tof;
        }
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // --- Hardware Initialization (omitted for brevity) ---
        shooter1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "motor2");
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelPIDF = new PIDFController(

                flywheelCoefficients[0], flywheelCoefficients[1],

                flywheelCoefficients[2], flywheelCoefficients[3]

        );

        turretPIDF = new PIDFController(

                turretCoefficients[0], turretCoefficients[1],

                turretCoefficients[2], turretCoefficients[3]

        );

        turretPIDF.setTolerance(1);

// Initialize targets

        setTargetVelocityTicks(0);

        setTurretTargetPosition(180);

        setHoodTargetAngle(45);
    }


    /**
     * Lookup table for angle to associated k (conversion factor) value
     */

    // ---------------------------------------------------------------------------
// MUZZLE VELOCITY LOOKUP TABLE (TUNING GUIDE)
// ---------------------------------------------------------------------------
// This table maps hood angle -> conversion factor k,
// where:  muzzleVelocity = k * flywheelTicksPerSec
//
// WHY WE NEED THIS:
// As the hood angle changes, the compression and friction change.
// This means the same flywheel RPM produces different muzzle exit speeds.
// A fixed k value causes distance errors, so we model k as a function of angle.
//
// HOW TO TUNE THESE VALUES:
// 1. Make a simple tuning OpMode that:
//      - Sets the hood to a known angle (e.g., 20°, 30°, 40°, 50°)
//      - Sets the flywheel to a fixed velocity (ex: 3000 ticks/sec)
//      - Lets you shoot one ring at a time
//
// 2. For each angle, place the robot at a known distance (ex: 3m).
//    Measure the ACTUAL muzzle velocity using high-speed video or a
//    field measurement (time-of-flight or distance travelled).
//
// 3. Compute k for that angle:
//        k = measuredMuzzleVelocity / commandedFlywheelTicksPerSec
//    Example:
//        measured = 14.8 m/s,
//        ticks/sec = 3000
//        k = 14.8 / 3000 = 0.00493
//
// 4. Update the table row with this k value.
//
// 5. Repeat for at least 4 angles across  hood's full range.
//
// 6. The interpolation function will smoothly fill the gaps in between.
//
// IMPORTANT:
// - All k values MUST increase slowly with angle (due to friction),
//   but the curve should be smooth — no sharp jumps.
// - Re-measure after changing flywheel grip, hood padding, or motor swaps.
// ---------------------------------------------------------------------------
    private static final double[][] MUZZLE_K_TABLE = {
            //  angleDeg,  k-factor
            {20, 0.150},  // TODO: replace with measured value
            {30, 0.157},
            {40, 0.166},
            {50, 0.172}
    };

    // Given a hood angle (in degrees), return the correct k-factor by
    // linearly interpolating between the nearest lookup table entries.
    private double getKForAngle(double angleDeg) {
        // --- Clamp below table range ---
        if (angleDeg <= MUZZLE_K_TABLE[0][0]) {
            return MUZZLE_K_TABLE[0][1];
        }

        // --- Clamp above table range ---
        if (angleDeg >= MUZZLE_K_TABLE[MUZZLE_K_TABLE.length - 1][0]) {
            return MUZZLE_K_TABLE[MUZZLE_K_TABLE.length - 1][1];
        }

        // --- Find the interval the angle belongs to ---
        for (int i = 0; i < MUZZLE_K_TABLE.length - 1; i++) {
            double a0 = MUZZLE_K_TABLE[i][0];
            double k0 = MUZZLE_K_TABLE[i][1];
            double a1 = MUZZLE_K_TABLE[i + 1][0];
            double k1 = MUZZLE_K_TABLE[i + 1][1];

            if (angleDeg >= a0 && angleDeg <= a1) {
                // how far between a0 and a1
                double t = (angleDeg - a0) / (a1 - a0);

                // Linear interpolation for smooth k value
                return k0 + t * (k1 - k0);
            }
        }

        // Should never occur
        return MUZZLE_K_TABLE[0][1];
    }

    /**
     * Normalizes an angle from [-180, 180] to [0, 360] (Robot Heading).
     */
    private static double normalizeRobotHeading0_360(double headingDeg) {
        return (headingDeg % 360 + 360) % 360;
    }

    /**
     * Translates an absolute Field Yaw (0-360) into the necessary Turret Encoder position (0-360).
     */
    private static double convertFieldYawToTurretEncoderTarget(double targetFieldYawDeg, double robotFieldYawDeg) {
        // 1. Convert Robot Heading from [-180, 180] to [0, 360]
        double robotHeading360 = normalizeRobotHeading0_360(robotFieldYawDeg);
        double targetHeading360 = normalizeRobotHeading0_360(targetFieldYawDeg);

        // 2. Calculate the raw difference (angle from the robot's front)
        double relativeAngle = targetHeading360 - robotHeading360;


        return (relativeAngle);
    }

    /**
     * Calculates the field-centric yaw angle required to hit a target.
     */
    public static double calculateAutoAlignYaw(double robotXInches, double robotYInches,
                                                double targetXInches, double targetYInches) {
        double deltaY = targetYInches - robotYInches;
        double deltaX = targetXInches - robotXInches;

        // Use Math.atan2(deltaX, deltaY) to correctly map standard (X=East, Y=North)
        // to the required FTC Yaw (0=North, 90=East)
        double targetFieldYawRad = Math.atan2(deltaX,deltaY);
        return (Math.toDegrees(targetFieldYawRad));
    }

    // ------------------------------------
    // ##  Turret Control
    // ------------------------------------
    public enum TurretMode {
        ROBOT_CENTRIC, FIELD_CENTRIC, AUTO_ALIGN
    }
    /**
     * Sets the turret's PID setpoint based on the selected mode.
     */
    private void setTurretTarget(double inputDeg, TurretMode mode, double currentTurretAngle0_360, double robotFieldYawDeg) {
        double rawNewTarget_0_360;

        switch (mode) {
            case FIELD_CENTRIC:
                // InputDeg is the absolute field yaw. We calculate the required encoder target.
                rawNewTarget_0_360 = currentTurretAngle0_360 + convertFieldYawToTurretEncoderTarget(inputDeg, robotFieldYawDeg);
                break;
            case ROBOT_CENTRIC:
                // InputDeg is the desired robot-relative angle (e.g., 180 for straight ahead).
                rawNewTarget_0_360 = inputDeg;
                break;
            case AUTO_ALIGN:
                // Conversion handled in setShooterTarget
                rawNewTarget_0_360 = inputDeg;
                break;
            default:
                rawNewTarget_0_360 = currentTurretAngle0_360;
                break;
        }


        setTurretTargetPosition(rawNewTarget_0_360);
    }

    public void setTurretTargetPosition(double positionDeg) {
        if (positionDeg > 360) {
            positionDeg = positionDeg % 360;
        }
        else if (positionDeg < 0) {
            positionDeg = Math.abs(positionDeg % 360);
        }
        targetTurretPosition = positionDeg;
        turretPIDF.setSetPoint(targetTurretPosition);
    }



    // ------------------------------------
    // ##  Flywheel Control
    // ------------------------------------

    public void setTargetVelocityTicks(double targetTicksPerSec) {
        targetFlywheelVelocity = Range.clip(targetTicksPerSec, 0, MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC);
        flywheelPIDF.setSetPoint(targetFlywheelVelocity);
    }

    // ------------------------------------
    // ##  Optimal Shot Search (T_min Optimization)
    // ------------------------------------

    /**
     * Iterates through a range of launch angles to find the combination of Velo and Angle
     * that results in the lowest Time of Flight (T_min) while respecting the
     * MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC constraint.
     */
    public OptimalShot calculateOptimalShot(double robotXInches, double robotYInches,
                                            double targetXInches, double targetYInches) {

        // --- 1. Geometric Setup ---
        double R = Math.hypot(targetXInches - robotXInches, targetYInches - robotYInches);
        double deltaY = (GOAL_HEIGHT_INCHES + GOAL_HEIGHT_SAFETY_OFFSET_INCHES)  - LAUNCH_HEIGHT_INCHES;

        if (R < 0.1) {
            return new OptimalShot(0.0, 90.0, 0.0);
        }

        // --- 2. Initialize Search Variables ---
        double bestTimeOfFlight = Double.MAX_VALUE;
        double optimalVeloTicks = 0.0;
        double optimalAngleDeg = 0.0;



        // Search bounds are now the dynamic launch angle constraints
        double searchMinAngle = MIN_LAUNCH_ANGLE_DEG;
        double searchMaxAngle = MAX_LAUNCH_ANGLE_DEG;

        if (searchMinAngle >= searchMaxAngle) {
            telemetry.addData("Shot Error", "Invalid Angle Range (Max <= Min)");
            return new OptimalShot(0.0, 0.0, -1.0);
        }

        // --- 3. Iterative Search for T_min within Constraints ---
        for (double alphaDeg = searchMinAngle; alphaDeg <= searchMaxAngle; alphaDeg += ANGLE_SEARCH_STEP_DEG) {

            double k = getKForAngle(alphaDeg);
            double V_muzzle_max_limit = MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC * k;

            double alphaRad = Math.toRadians(alphaDeg);



            // Calculate Required Velocity in In/Sec (V_req) for this specific angle
            double denominator = 2 * Math.pow(Math.cos(alphaRad), 2) * (R * Math.tan(alphaRad) - deltaY);

            if (denominator <= 0.001) {
                continue;
            }

            double V_req_sq = (GRAVITY_INCHES_PER_SEC_SQ * R * R) / denominator;
            double V_req_muzzle = Math.sqrt(V_req_sq);

            // --- 4. Apply Safety/Power Constraint ---
            if (V_req_muzzle > V_muzzle_max_limit) {
                continue;
            }

            // --- 5. Calculate Time of Flight (T_min) ---
            double T_current = calculateTOF(R, deltaY, V_req_muzzle, alphaRad);

            if (T_current > 0 && T_current < bestTimeOfFlight) {
                bestTimeOfFlight = T_current;
                optimalAngleDeg = alphaDeg;
                optimalVeloTicks = V_req_muzzle / k;
            }
        }

        // --- 6. Final Result Check ---
        if (bestTimeOfFlight == Double.MAX_VALUE) {
            telemetry.addData("Shot Error", "No safe shot found for distance: " + R);
            return new OptimalShot(0.0, 0.0, -1.0);
        }

        return new OptimalShot(optimalVeloTicks, optimalAngleDeg, bestTimeOfFlight);
    }

    // --- Helper for Time of Flight ---
    /**
     * Calculates the Time of Flight (T) based on horizontal motion.
     * NOTE: The V_muzzle used here was already calculated in the T_min search
     * to guarantee hitting the target height (deltaY).
     */
    private double calculateTOF(double R, double deltaY, double V_muzzle, double alpha_rad) {
//        double V_y_initial = V_muzzle * Math.sin(alpha_rad);
//        double discriminant = Math.pow(V_y_initial, 2) - 2 * GRAVITY_INCHES_PER_SEC_SQ * deltaY;
//
//        if (discriminant < 0) return -1.0;
//
//        double t_short = (V_y_initial - Math.sqrt(discriminant)) / GRAVITY_INCHES_PER_SEC_SQ;
//
//        if (t_short < 0) {
//            return (V_y_initial + Math.sqrt(discriminant)) / GRAVITY_INCHES_PER_SEC_SQ;
//        }
//        return t_short;


        // Horizontal component of velocity (V_x)
        double V_x = V_muzzle * Math.cos(alpha_rad);

        // If horizontal velocity is zero (vertical shot) or negative (impossible), return an error.
        if (V_x <= 0) {
            return -1.0;
        }

        // Time of Flight (T) = Horizontal Distance (R) / Horizontal Velocity (V_x)
        // The complex quadratic solve (discriminant) is not needed because the
        // V_muzzle is already guaranteed to solve the vertical equation.
        return R / V_x;

    }

    // ------------------------------------
    // ## ️ Set Target Wrapper
    // ------------------------------------

    /**
     * Calculates the ballistics (Velo, Angle) and applies controls (Turret, Flywheel, Hood).
     */
    public void setShooterTarget(
            double robotXInches, double robotYInches,
            double targetXInches, double targetYInches)
    {

        // 1. Always calculate ballistics (Velo/Angle) first, as this only needs R and DeltaY.
        OptimalShot shot = calculateOptimalShot(
                robotXInches, robotYInches,
                targetXInches, targetYInches);

        setTargetVelocityTicks(shot.requiredFlywheelTicks);
        setHoodTargetAngle(shot.requiredHoodAngle);
    }

    public void setShooterTarget(
            double robotXInches, double robotYInches,
            double targetXInches, double targetYInches,
            double currentTurretAngle0_360,
            double robotFieldYawDeg, // Input is [-180, 180]
            TurretMode turretMode, double turretInputDeg)
    {

        // 1. Always calculate ballistics (Velo/Angle) first, as this only needs R and DeltaY.
        OptimalShot shot = calculateOptimalShot(
                robotXInches, robotYInches,
                targetXInches, targetYInches);

        setTargetVelocityTicks(shot.requiredFlywheelTicks);
        setHoodTargetAngle(shot.requiredHoodAngle);

        double finalTurretInput = turretInputDeg;

        // 2. Set the Turret input based on the control mode priority:
        if (turretMode == TurretMode.FIELD_CENTRIC) {
            // A. Calculate the required absolute field yaw (last resort)
            double requiredFieldYaw = calculateAutoAlignYaw(robotXInches, robotYInches, targetXInches, targetYInches);

            // B. Convert this field yaw into the required turret encoder reading
            finalTurretInput = convertFieldYawToTurretEncoderTarget(requiredFieldYaw, robotFieldYawDeg);

        }

        // 3. Pass to the turret setter
        setTurretTarget(finalTurretInput, turretMode, currentTurretAngle0_360, robotFieldYawDeg);

//
    }



    // ------------------------------------
    // ##  Update Loop
    // ------------------------------------

    public void update(double currentFlywheelVelo,double currentTurretAngle0_360, double currentHoodAngleDeg) {
        flywheelVeloReached = flywheelPIDF.atSetPoint();
        turretReached = turretPIDF.atSetPoint();

        // Reset calibration flag at the start of the loop
        hoodCalibrationRequired = false;

        double flywheelOutput = flywheelPIDF.calculate(currentFlywheelVelo);
        flywheelOutput = Range.clip(flywheelOutput, 0, 1);
        setFlywheelPower(flywheelOutput);

        if (targetFlywheelVelocity == 0) {
            setFlywheelPower(0);
        }

        double turretOutput = turretPIDF.calculate(currentTurretAngle0_360);
        turretOutput = Range.clip(turretOutput, -maxTurretPower, maxTurretPower);
        turret.setPower(turretOutput);

        // Hood Control (No PIDF)
        moveHoodToPositionScale(targetHoodAngle);
        hoodReached = Math.abs(currentHoodAngleDeg - targetHoodAngle) <= 0.5;
    }

    // ------------------------------------
    // ##  Helpers
    // ------------------------------------

    public void moveHoodToPositionWithEncoder(double desiredAngle, double currentAngle) {


        double SERVO_STEP = 0.05;

        double SERVO_MIN = 0.0;

        double SERVO_MAX = 1.0;

        double ANGLE_TOLERANCE = 0.5;

        double error = desiredAngle - currentAngle;


        // 1. Exit if already at desired position
        if (Math.abs(error) < ANGLE_TOLERANCE) {
            return;
        }

        double pos = hoodServo.getPosition();
        double oldPos = pos; // Store for comparison

        // 2. Calculate new servo position and clip it
        pos += (error > 0) ? SERVO_STEP : -SERVO_STEP;
        pos = Range.clip(pos, SERVO_MIN, SERVO_MAX);

        if (pos == SERVO_MAX && oldPos != SERVO_MAX && error > ANGLE_TOLERANCE) {

            if (currentAngle < MAX_LAUNCH_ANGLE_DEG) {
                MAX_LAUNCH_ANGLE_DEG = currentAngle; // Update the upper launch angle limit
                hoodCalibrationRequired = true;
                }
        }

        if (pos == SERVO_MIN && oldPos != SERVO_MIN && error < -ANGLE_TOLERANCE) {

            if (currentAngle > MIN_LAUNCH_ANGLE_DEG) {
                MIN_LAUNCH_ANGLE_DEG = currentAngle; // Update the lower launch angle limit
                hoodCalibrationRequired = true;

            }
        }
        // 4. Set the new servo position
        hoodServo.setPosition(pos);
    }
    public double getHoodServoPositionInDegrees() {return hoodServo.getPosition() * 360;}
    public void moveHoodToPositionScale(double desiredAngle) {
        desiredAngle = Range.clip(desiredAngle, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
        double servoPos = hoodAngletoServoPos(desiredAngle);
        hoodServo.setPosition(servoPos);
    }
    public double hoodAngletoServoPos(double angle) {
        return Range.scale(angle, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG, HOOD_MIN_POS, HOOD_MIN_POS);
    }

    private static final double[][] angleLookUpTable = {
            {HOOD_MIN_ANGLE_DEG, HOOD_MIN_POS},
            {20, 0.2},
            {25, 0.25},
            {30, 0.3},
            {35, 0.35},
            {40, 0.4},
            {45, 0.45},
            {50, 0.5},
            {55, 0.55},
            {HOOD_MAX_ANGLE_DEG, HOOD_MAX_POS}
            //{angle, servopos}
    };
    public double moveHoodToPositionTable(double angle) {

        double clamped = Range.clip(angle,
                angleLookUpTable[0][0],
                angleLookUpTable[angleLookUpTable.length - 1][0]);

        double closestAngle = angleLookUpTable[0][0];
        double closestPos   = angleLookUpTable[0][1];
        double smallestDiff = Math.abs(clamped - closestAngle);

        for (int i = 1; i < angleLookUpTable.length; i++) {
            double tableAngle = angleLookUpTable[i][0];
            double diff = Math.abs(clamped - tableAngle);

            if (diff < smallestDiff) {
                smallestDiff = diff;
                closestAngle = tableAngle;
                closestPos   = angleLookUpTable[i][1];
            }
        }

        return closestPos;
    }

    public void setHoodTargetAngle(double target) {
        // Clips target angle against the dynamic launch constraints
        targetHoodAngle = Range.clip(target, MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG);
    }
    public double getFlywheelVelocity() {return (shooter1.getVelocity() + shooter2.getVelocity()) / 2;}

    public void setFlywheelPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void stopFlywheel() {
        setFlywheelPower(0);
        setTargetVelocityTicks(0);
    }
}