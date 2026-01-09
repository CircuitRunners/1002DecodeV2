package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;


public class BumAhhShooter {

    // ===== FLYWHEEL PIDF (Using your provided values) =====
    public static double f_kP = 0.000005;
    public static double f_kI = 0.0;
    public static double f_kD = 0.000000005;
    public static double f_kF = 0.0000027;

    // ===== TURRET PIDF (Using your provided values) =====
    public static double t_kP = 0.1;
    public static double t_kI = 0.0;
    public static double t_kD = 0.002;
    public static double t_kF = 0.0015;

    // ===== LINEAR REGRESSION TARGETS =====
    public static double VELO_SLOPE = 7.41, VELO_INTERCEPT = 802.23;
    public static double HOOD_SLOPE = 0.0027, HOOD_INTERCEPT = 0.066;

    // ===== CURRENT TARGETS =====
    public static double targetVelocity = 0;
    public static double targetTurretAngle = 0;
    public static double targetHoodServoPos = 0.2;

    // Hardware
    private DcMotorEx shooter1, shooter2, turret;
    private Servo hoodServo;
    private PIDFController flywheelPIDF, turretPIDF;

    public BumAhhShooter(HardwareMap hardwareMap) {
        // Flywheel motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turret motor
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hood servo
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        // Initialize PID objects once
        flywheelPIDF = new PIDFController(f_kP, f_kI, f_kD, f_kF);
        turretPIDF = new PIDFController(t_kP, t_kI, t_kD, t_kF);
    }

    /**
     * Primary loop for the shooter.
     * @param currentVelo Observed flywheel ticks/sec.
     * @param currentTurretAngle Observed turret encoder position/degrees.
     */
    public void update(double currentVelo, double currentTurretAngle) {
        // Sync PID coefficients with dashboard
        flywheelPIDF.setPIDF(f_kP, f_kI, f_kD, f_kF);
        turretPIDF.setPIDF(t_kP, t_kI, t_kD, t_kF);

        // Flywheel Control
        double fOutput = flywheelPIDF.calculate(currentVelo, targetVelocity);
        fOutput = Range.clip(fOutput, 0, 1.0);

        if (targetVelocity <= 10) {
            shooter1.setPower(0);
            shooter2.setPower(0);
        } else {
            shooter1.setPower(fOutput);
            shooter2.setPower(fOutput);
        }

        // Turret Control
        double tOutput = turretPIDF.calculate(currentTurretAngle, targetTurretAngle);
        tOutput = Range.clip(tOutput, -0.7, 0.7); // Clip turret power for safety
        turret.setPower(tOutput);

        // Hood Control
        hoodServo.setPosition(Range.clip(targetHoodServoPos, 0, 1));
    }

    public void setTargetsByDistance(double robotX, double robotY, double goalX, double goalY, double turretAngle) {
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Apply the distance formula: d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
        double distanceInches = Math.hypot(deltaX, deltaY);

        targetVelocity = (VELO_SLOPE * distanceInches) + VELO_INTERCEPT;
        targetHoodServoPos = (HOOD_SLOPE * distanceInches) + HOOD_INTERCEPT;
        targetTurretAngle = turretAngle;
    }

    public void stop() {
        targetVelocity = 0;
        targetTurretAngle = 0; // Optional: keeps turret at 0 or let it stay put
        shooter1.setPower(0);
        shooter2.setPower(0);
        turret.setPower(0);
    }
}