package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class BumAhhShooter {

    // ===== FLYWHEEL PIDF =====
    public static double f_kP = 0.000005, f_kI = 0.0, f_kD = 0.000000005, f_kF = 0.0000027;

    // ===== TURRET PIDF =====
    public static double t_kP = 0.1, t_kI = 0.0, t_kD = 0.002, t_kF = 0.0015;

    // ===== QUARTIC REGRESSION COEFFICIENTS (y = ax^4 + bx^3 + cx^2 + dx + e) =====
    // Velocity Coefficients
    public static double v_a = 0.00212656, v_b = -0.690055, v_c = 80.9096, v_d = -3021.17244, v_e = 221635.584;

    // Hood Coefficients
    public static double h_a = -0.00000272327, h_b = 0.000865664, h_c = -0.0980841, h_d = 4.82789, h_e = -51.50719;

    // ===== CURRENT TARGETS =====
    public static double targetVelocity = 0;
    public static double targetTurretAngle = 0;
    public static double targetHoodServoPos = 0.2;

    private DcMotorEx shooter1, shooter2, turret;
    private Servo hoodServo;
    private PIDFController flywheelPIDF, turretPIDF;

    public BumAhhShooter(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        flywheelPIDF = new PIDFController(f_kP, f_kI, f_kD, f_kF);
        turretPIDF = new PIDFController(t_kP, t_kI, t_kD, t_kF);
    }

    public void update(double currentVelo, double currentTurretAngle) {
        flywheelPIDF.setPIDF(f_kP, f_kI, f_kD, f_kF);
        turretPIDF.setPIDF(t_kP, t_kI, t_kD, t_kF);

        double fOutput = flywheelPIDF.calculate(currentVelo, targetVelocity);
        fOutput = Range.clip(fOutput, 0, 1.0);

        if (targetVelocity <= 10) {
            shooter1.setPower(0);
            shooter2.setPower(0);
        } else {
            shooter1.setPower(fOutput);
            shooter2.setPower(fOutput);
        }

        double tOutput = turretPIDF.calculate(currentTurretAngle, targetTurretAngle);
        tOutput = Range.clip(tOutput, -0.7, 0.7);
        turret.setPower(tOutput);

        hoodServo.setPosition(Range.clip(targetHoodServoPos, 0, 1));
    }

    /**
     * Calculates targets using quartic polynomial: ax^4 + bx^3 + cx^2 + dx + e
     */
    public void setTargetsByDistance(double robotX, double robotY, double goalX, double goalY, double turretAngle) {
        double x = Math.hypot(goalX - robotX, goalY - robotY); // distance

        // Quartic calculation for Velocity
        targetVelocity = (v_a * Math.pow(x, 4)) + (v_b * Math.pow(x, 3)) +
                (v_c * Math.pow(x, 2)) + (v_d * x) + v_e;

        // Quartic calculation for Hood
        targetHoodServoPos = (h_a * Math.pow(x, 4)) + (h_b * Math.pow(x, 3)) +
                (h_c * Math.pow(x, 2)) + (h_d * x) + h_e;

        targetTurretAngle = turretAngle;
    }

    public void stop() {
        targetVelocity = 0;
        shooter1.setPower(0);
        shooter2.setPower(0);
        turret.setPower(0);
    }
}