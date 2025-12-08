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

    // PIDF Coefficients (Update these for your robot)
    private static final double[] flywheelCoefficients = {0.002, 0, 0.0001, 0.000423};
    private static final double[] turretCoefficients = {0.01, 0, 0.0001, 0.005};

    // Target States
    private static double targetFlywheelVelocity = 0;   // Ticks/Sec
    private static double targetTurretPosition = 180;     // Degrees (0-360)
    private static double targetHoodAngle = 45;          // Degrees (0-90)
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

    //  CRITICAL CALIBRATION VALUE: Factor to convert Flywheel Ticks/Sec to Muzzle Velo In/Sec.
    private static final double FLYWHEEL_TICKS_TO_MUZZLE_VELO_FACTOR = 0.15; // PLACEHOLDER VALUE - TUNE THIS!

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
     * Normalizes an angle from [-180, 180] to [0, 360] (Robot Heading).
     */
    private static double normalizeRobotHeading0_360(double headingDeg) {
        if (headingDeg < 0) {
            return headingDeg *-1;
        }
        else if (headingDeg  > 0){
            return 360 - headingDeg;
        }
        return headingDeg;
    }

    /**
     * Translates an absolute Field Yaw (0-360) into the necessary Turret Encoder position (0-360).
     */
    private static double convertFieldYawToTurretEncoderTarget(double targetFieldYawDeg, double robotFieldYawDeg) {
        // 1. Convert Robot Heading from [-180, 180] to [0, 360]
        double robotHeading360 = normalizeRobotHeading0_360(robotFieldYawDeg);

        // 2. Calculate the raw difference (angle from the robot's front)
        double relativeAngle = targetFieldYawDeg - robotHeading360;


        return (relativeAngle);
    }

    /**
     * Calculates the field-centric yaw angle required to hit a target.
     */
    private static double calculateAutoAlignYaw(double robotXInches, double robotYInches,
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
        if (positionDeg > 360){
            positionDeg -= 360;
        }
        else if (positionDeg < 0){
            positionDeg +=360;
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

        double V_muzzle_max_limit = MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC * FLYWHEEL_TICKS_TO_MUZZLE_VELO_FACTOR;

        // Search bounds are now the dynamic launch angle constraints
        double searchMinAngle = MIN_LAUNCH_ANGLE_DEG;
        double searchMaxAngle = MAX_LAUNCH_ANGLE_DEG;

        if (searchMinAngle >= searchMaxAngle) {
            telemetry.addData("Shot Error", "Invalid Angle Range (Max <= Min)");
            return new OptimalShot(0.0, 0.0, -1.0);
        }

        // --- 3. Iterative Search for T_min within Constraints ---
        for (double alphaDeg = searchMinAngle; alphaDeg <= searchMaxAngle; alphaDeg += ANGLE_SEARCH_STEP_DEG) {

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
                optimalVeloTicks = V_req_muzzle / FLYWHEEL_TICKS_TO_MUZZLE_VELO_FACTOR;
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
    private double calculateTOF(double R, double deltaY, double V_muzzle, double alpha_rad) {
        double V_y_initial = V_muzzle * Math.sin(alpha_rad);
        double discriminant = Math.pow(V_y_initial, 2) - 2 * GRAVITY_INCHES_PER_SEC_SQ * deltaY;

        if (discriminant < 0) return -1.0;

        double t_short = (V_y_initial - Math.sqrt(discriminant)) / GRAVITY_INCHES_PER_SEC_SQ;

        if (t_short < 0) {
            return (V_y_initial + Math.sqrt(discriminant)) / GRAVITY_INCHES_PER_SEC_SQ;
        }
        return t_short;
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

    // ... (setShooterTarget overload remains the same) ...

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
        moveHoodToPosition(targetHoodAngle, currentHoodAngleDeg);
        hoodReached = Math.abs(currentHoodAngleDeg - targetHoodAngle) <= 0.5;
    }

    // ------------------------------------
    // ##  Helpers
    // ------------------------------------

    public void moveHoodToPosition(double desiredAngle, double currentAngle) {

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

    public void setHoodTargetAngle(double target) {
        // Clips target angle against the dynamic launch constraints
        targetHoodAngle = Range.clip(target, MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG);
    }

    public void setFlywheelPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void stopFlywheel() {
        setFlywheelPower(0);
        setTargetVelocityTicks(0);
    }
}