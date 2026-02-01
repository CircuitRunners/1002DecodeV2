package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Shooter {

    //NOTE FOR ALL - DEGREES INCREASE COUNTERCLOCKWISE LIKE UNIT CIRCLE
    private Telemetry telemetry;



    private static final double transferTimeSec = 0.0; //TUNE

//    public static double slotOneBLueAtan = 0;
//    public static double slotTwoBLueAtan = 0;


    // PIDF Coefficients
    public static  double[] flywheelCoefficients = {0.000005, 0, 0.000000005, 0.0000027};
//    private static final double[] turretCoefficients = {0.087, 0.000, 0.00399995, 0.0009};

    public static  double[] turretCoefficients = {0.9, 0.006, 0.035, 0.003};

    // Target States
    private static double targetFlywheelVelocity = 0;   // Ticks/Sec
    private static double targetTurretPosition = 0;
    private double ticksPerRevolution = 145.1;
    private double gearRatio = 149.0/15.0;// Degrees (0-360)
    private static double targetHoodAngle = 45;          // Degrees (0-90)

    private static double maxTurretPower = 0.8;

    // Hardware Constants
    //public static final int MOTOR_TICKS_PER_REV = 537; // Example value for a common motor

    // --- PHYSICS CONSTANTS ---
    private static final double GRAVITY = 386.088; // in/s^2

    // Heights (meters)
    private static final double LAUNCH_HEIGHT_IN = 16; //11.6
    private static final double GOAL_HEIGHT_IN = 38.75;
    private static final double GOAL_HEIGHT_SAFETY_OFFSET_IN = 2.0;

    final double GOAL_HEIGHT = 38.75;          // inches
    final double ANGLE_BIAS_DEG = -18.35;       // a
    final double G_INCHES = 386.088;            // gravity
    final double VECTOR_TUNE = 2.49;

    private static final double EFFECTIVE_GOAL_HEIGHT_IN =
            GOAL_HEIGHT_IN + GOAL_HEIGHT_SAFETY_OFFSET_IN;

    // Effective height we want the ball to reach


    // --- SHOT CONSTRAINTS (For Iterative Search) ---
    private static final double MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC = 400000;


    private static double MIN_LAUNCH_ANGLE_DEG = 5.0;
    private static double MAX_LAUNCH_ANGLE_DEG = 45.0;

    private static final double ANGLE_SEARCH_STEP_DEG = 2.5;

    // Status flags
    public static boolean flywheelVeloReached = false;
    public static boolean turretReached = false;
    public static boolean hoodReached = false;

    //public static boolean isShotImpossible = false;


    public boolean hoodCalibrationRequired = false;

    // Hardware
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx turret;
    private Servo hoodServo;

    private PIDFController flywheelPIDF;
    private PIDFController turretPIDF;

    private DigitalChannel shooterBeamBreak;

    // Velocity Coefficients
//    public static double v_a = 0.00212656, v_b = -0.690055, v_c = 80.9096, v_d = -3021.17244, v_e = 221635.584;
//    public static double v_a = -0.000419907, v_b = 0.162408, v_c = -21.9297, v_d = 1991.28761, v_e = 150460.825;


    // Hood Coefficients
    //public static double h_a = -0.00000272327, h_b = 0.000865664, h_c = -0.0980841, h_d = 4.82789, h_e = -51.50719;
    public static double h_a = -(2.52835e-7), h_b = 0.00012437, h_c = -0.0230779, h_d = 1.9269, h_e = -17.60642;

    /**
     * Flywheel velocity lookup table
     *
     * distanceInches , flywheelTicksPerSec
     *
     * MUST be sorted by distance (ascending)
     */
    private static final double[][] VELO_LUT = {
            { 20.0, 182746 },
            { 25.0, 188910 },
            { 30.0, 194507 },
            { 40.0, 204343 },
            { 50.0, 212877 },
            { 60.0, 220628 },
            { 70.0, 228019 },
            { 80.0, 235366 },
            { 90.0, 242891 },
            { 100.0, 250709 },
            { 110.0, 258839 },
            { 120.0, 267196 },
            { 130.0, 275569 },
            { 140.0, 283754 },
            { 150.0, 291284 },
            { 155.0, 292000 },
    };

    /**
     * Data structure for the calculated optimal shot parameters.
     * Contains only ballistics data (velocity, angle, time).
     * NOTE: Velocity is in Ticks/Sec.
     */


    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // --- Hardware Initialization (omitted for brevity) ---
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterBeamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        shooterBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        flywheelPIDF = new PIDFController(

                flywheelCoefficients[0], flywheelCoefficients[1],

                flywheelCoefficients[2], flywheelCoefficients[3]

        );

        //flywheelPIDF.setTolerance(4500);

        turretPIDF = new PIDFController(

                turretCoefficients[0], turretCoefficients[1],

                turretCoefficients[2], turretCoefficients[3]

        );

        turretPIDF.setTolerance(2);


// Initialize targets

        turretPIDF.setSetPoint(0);
        flywheelPIDF.setSetPoint(0);


        setHoodTargetAngle(45);
    }


    /**
     * Lookup table for angle to associated k (conversion factor) value
     */
/**
 * Calibrated muzzle velocity conversion table
 *
 * k = muzzleVelocity (in/s) / flywheelTicksPerSec
 *
 * Tuned using real shot data with an effective launch height of ~16 in.
 * These values intentionally absorb:
 *  - ball compression
 *  - hood friction
 *  - spin lift
 *
 * DO NOT try to "fix" these with physics — trust the table.
 */
//private static final double[][] MUZZLE_K_TABLE = {
//        // angleDeg ,   k
//        { 38.0, 0.00098 },
//        { 42.0, 0.00092 },
//        { 44.0, 0.00090 },
//        { 45.0, 0.00090 }
//};
//    // ---------------------------------------------------------------------------
////
//
//    // Given a hood angle (in degrees), return the correct k-factor by
//    // linearly interpolating between the nearest lookup table entries.
//    private double getKForAngle(double angleDeg) {
//        // --- Clamp below table range ---
//        if (angleDeg <= MUZZLE_K_TABLE[0][0]) {
//            return MUZZLE_K_TABLE[0][1];
//        }
//
//        // --- Clamp above table range ---
//        if (angleDeg >= MUZZLE_K_TABLE[MUZZLE_K_TABLE.length - 1][0]) {
//            return MUZZLE_K_TABLE[MUZZLE_K_TABLE.length - 1][1];
//        }
//
//        // --- Find the interval the angle belongs to ---
//        for (int i = 0; i < MUZZLE_K_TABLE.length - 1; i++) {
//            double a0 = MUZZLE_K_TABLE[i][0];
//            double k0 = MUZZLE_K_TABLE[i][1];
//            double a1 = MUZZLE_K_TABLE[i + 1][0];
//            double k1 = MUZZLE_K_TABLE[i + 1][1];
//
//            if (angleDeg >= a0 && angleDeg <= a1) {
//                // how far between a0 and a1
//                double t = (angleDeg - a0) / (a1 - a0);
//
//                // Linear interpolation for smooth k value
//                return k0 + t * (k1 - k0);
//            }
//        }
//
//        // Should never occur
//        return MUZZLE_K_TABLE[0][1];
//    }

    /**
     * Normalizes an angle from [-180, 180] to [0, 360] (Robot Heading).
     */
    private static double normalizeRobotHeading0_360(double headingDeg) {
//        if (headingDeg < 0) {
//            return headingDeg *-1;
//        }
//        else if (headingDeg  > 0){
//            return 360 - headingDeg;
//        }
//        return headingDeg;
        double normalized = headingDeg % 360;
        normalized *= -1;
        if (normalized < 0) normalized += 360;
        return normalized;
    }


    private static double calculateAutoAlignYaw(double robotXInches, double robotYInches,
                                                double targetXInches, double targetYInches, boolean isRed) {
        double deltaY = targetYInches - robotYInches;
        double deltaX = targetXInches - robotXInches;

        // Standard atan2(y, x) for East = 0, North = 90
        double targetFieldYawRad = Math.atan2(-deltaY, deltaX);
        double targetFieldYawRadBlue = Math.atan2(deltaX, -deltaY);

        double targetFieldYawDeg = Math.toDegrees(targetFieldYawRad);
        if (targetFieldYawDeg < 0) {
            targetFieldYawDeg += 360;
        }
        double targetFieldYawDegBlue = Math.toDegrees(targetFieldYawRadBlue);
        if (targetFieldYawDegBlue < 0) {
            targetFieldYawDegBlue += 360;
        }
        if (isRed) {
            return Math.round(targetFieldYawDeg);
        }
        return Math.round(targetFieldYawDegBlue); // Returns (-180 to 180)
    }

//    private static double calculateAutoAlignYaw(double robotX, double robotY, double goalX, double goalY, boolean isRed) {
//        double deltaX = goalX - robotX;
//        double deltaY = goalY - robotY;
//
//        // 0° = +X, CCW positive
//        double yaw = Math.toDegrees(Math.atan2(deltaY, deltaX));
//
//        yaw = Math.round(yaw * 10.0)/10.0;
//
//        if (yaw < 0) yaw += 360;
//
//
//
//        // Turret zero = robot forward (+Y)
//        yaw = (yaw - 90 + 360) % 360;
//
//        return yaw;
//    }



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
                // absoluteTarget = normalizeRobotHeading0_360(inputFieldDeg - robotFieldYawDeg);
                break;

            case AUTO_ALIGN:
                // Input is already the field yaw calculated from (dx, dy)
                // Example: calculateAutoAlignYaw(robotX, robotY, targetX, targetY)
                absoluteTarget = Range.clip((((inputFieldDeg - robotFieldYawDeg + 360) % 360)), 0, 360);
                // absoluteTarget = normalizeRobotHeading0_360(inputFieldDeg - robotFieldYawDeg);
                break;

            case ROBOT_CENTRIC:
                // Directly setting the turret 0-360 relative to the front of the robot
                absoluteTarget = Range.clip(inputFieldDeg, -360, 360);
                break;
        }



        // Set the setpoint (Non-Continuous PID handles the 'long way')
        setTurretTargetPosition(absoluteTarget + mannualTurretAdjust);
    }
    public void setTurretTargetPosition(double positionDeg) {
//        if (positionDeg > 360){
//            positionDeg -= 360;
//        }
//        else if (positionDeg < 0){
//            positionDeg +=360;
//        }
        // targetTurretPosition = Range.clip(positionDeg, -265, 265);
        //targetTurretPosition = Range.clip(positionDeg,0,315);



        // 2. Map 0 -> 360 into -180 -> 180
        // If input is 0..180, it stays 0..180
        // If input is 181..360, it becomes -179..0
        double physicalTarget;
        if (positionDeg <= 180) {
            physicalTarget = positionDeg;
        } else {
            physicalTarget = positionDeg - 360;
        }

        // --- WIRIG DEAD ZONE HANDLING ---
        // If target is in the 315-360 deadzone, pick the closest safe limit
        if (Math.abs(physicalTarget) >= 178) {
            if (physicalTarget > 0) { // Closer to 0
                physicalTarget = physicalTarget;
            } else { // Closer to 315
                physicalTarget = -178;
            }
        }

        // 3. Clip to stay away from the physical hardstops (e.g., +/- 175)
        targetTurretPosition = Range.clip(physicalTarget, -178, 180);
    }



    // ------------------------------------
    // ##  Flywheel Control
    // ------------------------------------

    public void setTargetVelocityTicks(double targetTicksPerSec) {
        targetFlywheelVelocity = Range.clip(targetTicksPerSec, 0, MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC);
        //flywheelPIDF.setSetPoint(targetFlywheelVelocity);
    }

    // ------------------------------------
    // ##  Optimal Shot Search (T_min Optimization)
    // ------------------------------------

    /**
     * Iterates through a range of launch angles to find the combination of Velo and Angle
     * that results in the lowest Time of Flight (T_min) while respecting the
     * MAX_FLYWHEEL_VELO_LIMIT_TICKS_SEC constraint.
     */


    // ------------------------------------
    // ## ️ Set Target Wrapper
    // ------------------------------------




    /* ========================================================= */
    /* ========== ROBOT VELOCITY ALONG SHOT DIRECTION ============ */
    /* ========================================================= */

    // --- Physical Constants (inches & Seconds) ---
    private static final double TICKS_PER_REV_ENCODER = 4096.0;

    private static final double GEAR_RATIO = 33.0 / 27.0;

    // Variables outside the loop to persist "memory" between frames
    private double persistentShotTime = 0.0;
    private double finalAdjustedVeloTicks = 0;
    private double finalAdjustedHoodDeg = 0;
    private double finalAdjustedTurretFieldYaw = 0;


    //pass in everything in inches and degrees (inch/sec for velo too)
//    public void calculateIterativeShot(
//            double robotX, double robotY, double goalX, double goalY,
//            double robotVeloX, double robotVeloY,double robotHeading, boolean isRed, double flywheelMannualOffset, double hoodMannualOffset, double turretManualOffset) {
//
//        // Start with the real goal position
//        double virtualGoalX = goalX;
//        double virtualGoalY = goalY;
//
//        // Run 5 iterations to converge on the moving target
//        for (int i = 0; i < 1; i++) {
//            // 1. Calculate distance to our "Virtual Goal"
//            double dist = Math.hypot(virtualGoalX - robotX, virtualGoalY - robotY);
//
//            // 2. Get base Ticks and Hood from your Quartic curves for this distance
//            double baseTicks = (v_a * Math.pow(dist, 4)) + (v_b * Math.pow(dist, 3)) +
//                    (v_c * Math.pow(dist, 2)) + (v_d * dist) + v_e;
//
//            double baseHood = (h_a * Math.pow(dist, 4)) + (h_b * Math.pow(dist, 3)) +
//                    (h_c * Math.pow(dist, 2)) + (h_d * dist) + h_e;
//
//            // 3. Convert Ticks to Inches/Sec to find Time of Flight
//            // Formula: Ticks -> Inches/Sec -> calculateTimeToGoalSeconds
//            double ticksToInches =
//                    calcFlywheelSpeedInches(baseTicks);
//            double muzzleVeloInches = baseTicks * ticksToInches;
//
//            persistentShotTime = calculateTimeToGoalSeconds(muzzleVeloInches, baseHood,dist);
//
//            // 4. Update the Virtual Goal position based on robot velocity
//            // New Pos = Current Pos + (-Robot Velocity * Time)
//            virtualGoalX = goalX - (robotVeloX * (persistentShotTime + transferTimeSec));
//            virtualGoalY = goalY - (robotVeloY * (persistentShotTime + transferTimeSec));
//
//            // 5. Store the final results from this iteration
//            finalAdjustedVeloTicks = baseTicks;
//            finalAdjustedHoodDeg = baseHood;
//            finalAdjustedTurretFieldYaw = calculateAutoAlignYaw(robotX, robotY, virtualGoalX, virtualGoalY, isRed);
//        }
//
//        // After the loop, apply the results to the hardware
//        setTargetVelocityTicks(finalAdjustedVeloTicks + flywheelMannualOffset);
//        setHoodTargetAngle(Range.clip(finalAdjustedHoodDeg + hoodMannualOffset, 0, 45));
//        // Turret uses the yaw calculated for the VIRTUAL goal
//        setTurretTarget(finalAdjustedTurretFieldYaw, TurretMode.AUTO_ALIGN, robotHeading,turretManualOffset);
//    }





    private static double calculateTimeToGoalSeconds(
            double exitVelocityInchesPerSec,
            double hoodAngleDegrees, double currentDistance) {

        // Convert angle to radians for trig
        double hoodRad = Math.toRadians(hoodAngleDegrees);

        double vX = exitVelocityInchesPerSec * Math.cos(hoodRad);
        double vY = exitVelocityInchesPerSec * Math.sin(hoodRad);

// Solve vertical motion for height constraint (as you already do)
        double deltaHeightIn = EFFECTIVE_GOAL_HEIGHT_IN - LAUNCH_HEIGHT_IN;

        double tHeight = (vY + Math.sqrt(vY*vY + 2*GRAVITY*deltaHeightIn)) / GRAVITY;

// Solve horizontal motion for distance
        double horizontalDistanceIn = currentDistance; // pass this in
        double tHorizontal = horizontalDistanceIn / vX;

// FINAL TOF = max of the two
        return Math.max(tHeight, tHorizontal);
    }
//




    // ------------------------------------
    // ##  Update Loop
    // ------------------------------------

    public void update(double currentTurretAngle0_360) {
        double shooter1Velocity = shooter1.getVelocity();
        double shooter2Velocity = shooter2.getVelocity();

        double averageVelo = (shooter1Velocity + shooter2Velocity) / 2;

        if (averageVelo >= targetFlywheelVelocity - 300 || averageVelo <= targetFlywheelVelocity + 300) {
            flywheelVeloReached = true;
        }
        else {
            flywheelVeloReached = false;
        }

        if (currentTurretAngle0_360 >= targetTurretPosition - 0.7 || currentTurretAngle0_360 <= targetTurretPosition + 0.7) {
            turretReached = true;
        }
        else {
            turretReached = false;
        }



        // Reset calibration flag at the start of the loop
        hoodCalibrationRequired = false;


        flywheelPIDF.setPIDF(flywheelCoefficients[0],flywheelCoefficients[1],flywheelCoefficients[2],flywheelCoefficients[3]);

        double flywheelOutput = flywheelPIDF.calculate(averageVelo,targetFlywheelVelocity);
        flywheelOutput = Range.clip(flywheelOutput, 0, 1);
        shooter1.setPower(flywheelOutput);
        shooter2.setPower(flywheelOutput);

//        if (targetFlywheelVelocity == 0) {
//            setFlywheelPower(0);
//        }
        turretPIDF.setPIDF( turretCoefficients[0], turretCoefficients[1],

                turretCoefficients[2], turretCoefficients[3]);
        double turretOutput = turretPIDF.calculate(currentTurretAngle0_360,targetTurretPosition);

        if (Math.abs(targetTurretPosition - currentTurretAngle0_360) <= 4){
            turretOutput = 0;
        }
        else{
            turretOutput = Range.clip(turretOutput, -maxTurretPower, maxTurretPower);
        }
        turret.setPower(turretOutput);

        // Hood Control (No PIDF)
        hoodServo.setPosition(hoodAngleToServoPos(targetHoodAngle));
        hoodReached = Math.abs(getHoodServoPositionInDegrees(hoodServo.getPosition()) - targetHoodAngle) <= 1.5;
    }


    public void manualTurretOverride(double power, double currentAngle) {
        // 1. Set the power directly
        turret.setPower(Range.clip(power,-0.6,0.6));

        // 2. Update the setpoint to the current angle to "calm" the PID
        targetTurretPosition = currentAngle;
       // turretPIDF.setSetPoint(targetTurretPosition);
    }
    // ------------------------------------
    // ##  Helpers
    // ------------------------------------



    /**
     * Returns interpolated flywheel velocity (ticks/sec)
     * for a given distance in inches.
     */
    private static double getFlywheelVeloFromDistanceLUT(double distanceInches) {

        // --- Clamp below table ---
        if (distanceInches <= VELO_LUT[0][0]) {
            return VELO_LUT[0][1];
        }

        // --- Clamp above table ---
        if (distanceInches >= VELO_LUT[VELO_LUT.length - 1][0]) {
            return VELO_LUT[VELO_LUT.length - 1][1];
        }

        // --- Find the two surrounding points ---
        for (int i = 0; i < VELO_LUT.length - 1; i++) {
            double d0 = VELO_LUT[i][0];
            double v0 = VELO_LUT[i][1];

            double d1 = VELO_LUT[i + 1][0];
            double v1 = VELO_LUT[i + 1][1];

            if (distanceInches >= d0 && distanceInches <= d1) {
                // interpolation factor (0 → 1)
                double t = (distanceInches - d0) / (d1 - d0);

                // linear interpolation
                return v0 + t * (v1 - v0);
            }
        }

        // Should never hit
        return VELO_LUT[0][1];
    }
    /**
     * Maps a desired hood angle to the required servo position [0.0, 1.0] using linear scaling.
     */
    private double hoodAngleToServoPos(double angle) {
        // Map the input angle range to the output servo position range
        return Range.scale(
                angle,
                MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG, // Input Range
                0.2, 0.025              // Output Range
        );
    }

    /**
     * Maps the current servo position [0.0, 1.0] back to the current hood angle in degrees.
     */
    private double getHoodServoPositionInDegrees(double currentServoPos) {
        // Map the servo position range back to the hood angle range (INVERSE SCALING)
        return Range.scale(
                currentServoPos,
                0.2, 0.025,              // Input Range
                MIN_LAUNCH_ANGLE_DEG, MAX_LAUNCH_ANGLE_DEG   // Output Range (CORRECTED)
        );
    }


//    private double getXVelocityOffset(double robotVeloX, double currentGoalX, double TOF){
//        double finalGoalPos = (-robotVeloX * TOF) + currentGoalX;
//        return finalGoalPos;
//    }
//
//    private double getYVelocityOffset(double robotVeloY, double currentGoalY, double TOF){
//        double finalGoalPos = (-robotVeloY * TOF) + currentGoalY;
//        return finalGoalPos;
//
//    }
//    private double getTurretAngle() {
//        double currentTicks = turret.getCurrentPosition();
//        return ((currentTicks / ticksPerRevolution) / gearRatio) * 360;
//    }

    public void setHoodTargetAngle(double angle){
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
        if (shooterBeamBreak.getState() == false){
            return true;
        }
        return false;
    }


//   public double flywheelTicksFromDistance(double robotX, double robotY, double goalX, double goalY) {
//        double distanceInches = Math.hypot(goalX - robotX, goalY - robotY);
//        // --- constants (copied from your OpMode) ---
//        // tune factor
//
//        // Prevent nonsense
//        distanceInches = Math.max(distanceInches, 1.0);
//
//        // Initial physics angle
//        double angle = Math.atan(
//                2 * GOAL_HEIGHT / distanceInches
//                        - Math.tan(Math.toRadians(ANGLE_BIAS_DEG))
//        );
//
//        // Clamp to hood limits
//        angle = MathFunctions.clamp(
//                angle,
//                Math.toRadians(10),
//                Math.toRadians(85)
//        );
//
//        double cos = Math.cos(angle);
//        double denom = 2 * cos * cos *
//                (distanceInches * Math.tan(angle) - GOAL_HEIGHT);
//
//        if (denom <= 0) return 0;
//
//        // Muzzle velocity (in/s)
//        double velocity = Math.sqrt(
//                G_INCHES * distanceInches * distanceInches / denom
//        );
//
//        // Convert to ticks
//        return calcFlywheelSpeedTicks(
//                velocity * VECTOR_TUNE
//        );
//    }

    static final double TICKS_PER_REV = 4096.0;
    static final double WHEEL_RADIUS_IN = 1.4173; // 72mm / 2 -> inches
    static final double G = 386.0; // in/s^2
    public static double k = 0.4;

    public static double calculateShotTime(
            double ticksPerSec,
            double hoodAngleRad,
            double GOAL_X, double GOAL_Y,double robotX,double robotY
    ) {



        double distanceInches = Math.hypot(GOAL_X - robotX, GOAL_Y - robotY);


        double v0 =
                k *
                        (2.0 * Math.PI * WHEEL_RADIUS_IN * ticksPerSec)
                        / TICKS_PER_REV;

        double vx = v0 * Math.cos(hoodAngleRad);
        return distanceInches / vx;
    }
    public void setTargetsByDistance(double robotX, double robotY, double goalX, double goalY, double robotAngle, boolean autoAlign, double hoodMannualAdjustment, boolean isRed, double turretAdjustment) {
        double x = Math.hypot(goalX - robotX, goalY - robotY); // distance

        double hoodPos;
        double velo;


        velo = getFlywheelVeloFromDistanceLUT(x);
        // Quartic calculation for Velocity
//        velo = (v_a * Math.pow(x, 4)) + (v_b * Math.pow(x, 3)) +
//                (v_c * Math.pow(x, 2)) + (v_d * x) + v_e;

       // velo = flywheelTicksFromDistance(robotX,robotY,goalX,goalY);


        // Quartic calculation for Hood
        hoodPos = (h_a * Math.pow(x, 4)) + (h_b * Math.pow(x, 3)) +
                (h_c * Math.pow(x, 2)) + (h_d * x) + h_e;

        setTargetVelocityTicks(velo);
        setHoodTargetAngle(Range.clip(hoodPos + hoodMannualAdjustment,0,45));
        if (autoAlign){

            double requiredFieldYaw;
            if (isRed) {
                requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY,true);
            }

            else {
                requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY,false);
            }
            // B. Pass to the turret setter
            setTurretTarget(requiredFieldYaw, TurretMode.AUTO_ALIGN, robotAngle,turretAdjustment);
        }
    }



    public void setTargetsByDistanceAdjustable(double robotX, double robotY, double goalX, double goalY, double robotAngle, boolean autoAlign,double flywheelMannualAdjustment, double hoodMannualAdjustment, boolean isRed, double turretManualAdjustment) {
        double x = Math.hypot(goalX - robotX, goalY - robotY); // distance

        double hoodPos;
        double velo;


        velo = getFlywheelVeloFromDistanceLUT(x);
        // Quartic calculation for Velocity
//        velo = (v_a * Math.pow(x, 4)) + (v_b * Math.pow(x, 3)) +
//                (v_c * Math.pow(x, 2)) + (v_d * x) + v_e;
       // velo = flywheelTicksFromDistance(robotX,robotY,goalX,goalY);



        // Quartic calculation for Hood
        hoodPos = (h_a * Math.pow(x, 4)) + (h_b * Math.pow(x, 3)) +
                (h_c * Math.pow(x, 2)) + (h_d * x) + h_e;

        setTargetVelocityTicks(velo + flywheelMannualAdjustment);
        setHoodTargetAngle(Range.clip(hoodPos + hoodMannualAdjustment,0,45));
        if (autoAlign){
            double requiredFieldYaw;
            if (!isRed) {
                requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY + 3, false);
            }
            else { requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY, true);}
            // B. Pass to the turret setter
            setTurretTarget(requiredFieldYaw, TurretMode.AUTO_ALIGN, robotAngle,turretManualAdjustment);
        }
    }

    public double getCurrentTurretPosition(){
        return Range.scale(
                turret.getCurrentPosition(),
                -720, 720,              // Input Range
                -180, 180
        );// Outp// ut Range (CORRECTED)
    }







    public void rezeroTurretPosition(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setTurretTarget(0,TurretMode.ROBOT_CENTRIC,getCurrentTurretPosition(),0);
    }

    public double getTargetFLywheelVelo(){
        return targetFlywheelVelocity;
    }

    public double calcFlywheelSpeedInches(double flywheelSpeedTicks){
        return flywheelSpeedTicks * ((72 * Math.PI)/(4096 * 25.4));
    }
    public double calcFlywheelSpeedTicks(double flywheelSpeedInches){
        return flywheelSpeedInches / ((72 * Math.PI)/(4096 * 25.4));
    }

    public void resetTurretPID(){
        turretPIDF.reset();
    }

    public double getFlywheelVelo(){
        return (shooter1.getVelocity() + shooter2.getVelocity())/2;
    }



//    public double getFlywheelVelo(){
//        return shooter1.getVelocity();
//    }




}
//wassup