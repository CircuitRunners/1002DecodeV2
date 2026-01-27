//package org.firstinspires.ftc.teamcode.Testers;
//
//import static org.firstinspires.ftc.teamcode.Testers.notCookedTurretTuner.cookedLoopTargetMS;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.MathFunctions;
//import com.pedropathing.math.Vector;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.Config.Util.Poses;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.List;
//import java.util.Locale;
//
//@Configurable
//@TeleOp(name = "EXACT MATH COMPARISON", group = "TEST")
//public class ShootingWhileMovingTester extends OpMode {
//
//    // Target Constants
//    public static double RED_GOAL_X = 127.0;
//    public static double BLUE_GOAL_X = 17.0;
//    public static double GOAL_Y = 130.5;
//
//    public static double VECTOR_TUNE_FACTOR = 2.54;
//
//    private ElapsedTime loopTimer = new ElapsedTime();
//
//    // Subsystems
//    private GamepadEx player1;
//    private MecanumDrive drive;
//    private Follower follower;
//    private GoBildaPinpointDriver pinpoint;
//    private List<LynxModule> allHubs;
//    private Shooter shooterLogic;
//    private Sensors sensors;
//
//    private Intake intake;
//
//
//
//
//    private boolean enableShooter = false;
//    private boolean isRedAlliance = true;
//
//    double cookedLoopTargetMs = 40;
//
//    public static double a = -18.35;
//
//    // Quartic Coefficients from your Shooter class
//    private final double v_a = -0.000419907, v_b = 0.162408, v_c = -21.9297, v_d = 1991.28761, v_e = 150460.825;
//    private final double h_a = -(2.52835e-7), h_b = 0.00012437, h_c = -0.0230779, h_d = 1.9269, h_e = -17.60642;
//
//    @Override
//    public void init() {
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(Poses.getStartingPose());
//
//        drive = new MecanumDrive();
//        drive.init(hardwareMap);
//
//        intake = new Intake(hardwareMap, telemetry);
//
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        configurePinpoint();
//
//        player1 = new GamepadEx(gamepad1);
//        shooterLogic = new Shooter(hardwareMap, telemetry);
//
//        sensors = new Sensors();
//        sensors.init(hardwareMap, "SRSHub");
//
//        if(Poses.getAlliance() != null) {
//            isRedAlliance = (Poses.getAlliance() == Poses.Alliance.RED);
//        }
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) hub.clearBulkCache();
//        player1.readButtons();
//        pinpoint.update();
//        follower.update();
//        sensors.update();
//        shooterLogic.update(sensors.getFlywheelVelo(), shooterLogic.getCurrentTurretPosition());
//        intake.update(shooterLogic.isBeamBroken(), LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE, sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
//                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
//                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));
//
//        // --- DRIVING (v2Teleop Logic) ---
//        double forward = player1.getLeftY();
//        double strafe = player1.getLeftX();
//        double rotate = player1.getRightX();
//
//        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }
//        double heading = follower.getPose().getHeading();
//        double theta = Math.atan2(forward, strafe) - heading;
//        double r = Math.hypot(forward, strafe);
//        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
//
//        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
//            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
//        }
//
//
//        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)){
//            if (!enableShooter){
//                enableShooter = true;
//            }
//            else {
//                enableShooter = false;
//            }
//        }
//
//        if (gamepad1.right_trigger > 0.2){
//            intake.doTestShooter();
//        }
//        else {
//            intake.doIntakeHalt();
//        }
//
//        // --- RUN CALCULATIONS ---
//        CalculationResult vectorRes = calculateShotVectorExact(heading);
//        CalculationResult quarticRes = calculateQuarticMath(follower.getPose());
//
//        if (enableShooter){
//            shooterLogic.setTargetVelocityTicks(vectorRes.ticks + 850);
//            shooterLogic.setHoodTargetAngle(vectorRes.hood);
//            shooterLogic.setTurretTargetPosition(vectorRes.turret);
//        }
//        else if (!enableShooter){
//            shooterLogic.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC,heading,0);
//            shooterLogic.stopFlywheel();
//        }
//
//
//
//
//
//        // --- TELEMETRY ---
//        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
//
//        telemetry.addLine("\n=== VECTOR METHOD (EXACT) ===");
//        telemetry.addData("V-Ticks", "%.1f", vectorRes.ticks);
//        telemetry.addData("V-Hood", "%.2f", vectorRes.hood);
//        telemetry.addData("V-Turret", "%.2f", vectorRes.turret);
//
//        telemetry.addLine("\n=== QUARTIC METHOD (EXACT) ===");
//        telemetry.addData("Q-Ticks", "%.1f", quarticRes.ticks);
//        telemetry.addData("Q-Hood", "%.2f", quarticRes.hood);
//        telemetry.addData("Q-Turret", "%.2f", quarticRes.turret);
//
//        double loopTime = loopTimer.milliseconds();
//
//        telemetry.addData("loop time", loopTime);
//
//        double remaining = cookedLoopTargetMs - loopTime;
////        if (remaining > 0) {
////            try {
////                Thread.sleep((long) remaining);
////            } catch (InterruptedException ignored) {}
////        }
//
//
//
//        telemetry.update();
//
//        loopTimer.reset();
//    }
//
//    /**
//     * Re-implemented EXACTLY from your original provided snippet
//     */
//    private CalculationResult calculateShotVectorExact(double robotHeading) {
//        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
//        Vector robotToGoalVector = new Vector();
//        robotToGoalVector.setOrthogonalComponents(targetX - follower.getPose().getX(), GOAL_Y - follower.getPose().getY());
//
//        double g = 386.088;
//        double x = Math.max(robotToGoalVector.getMagnitude() - 5, 1.0);
//        double y = (38.75 - 16) + 3;
//
//        // START HERE: Use a very simple, neutral entry angle.
//        // As you get closer (x small), 2y/x grows, making the angle steeper.
//        double aLocal = Math.toRadians(a);
//
//        // 1. Solve Physics Angle (0 is flat, 90 is vertical)
//        // IMPORTANT: Allow this to solve for the FULL physical range (10 to 85 degrees)
//        double physicsAngleRad = Math.atan(2 * y / x - Math.tan(aLocal));
//        physicsAngleRad = MathFunctions.clamp(physicsAngleRad, Math.toRadians(10), Math.toRadians(85));
//
//        // 2. Velocity Solve (Original stable logic)
//        double denom = (2 * Math.pow(Math.cos(physicsAngleRad), 2) * (x * Math.tan(physicsAngleRad) - y));
//        double flywheelSpeed = (denom > 0) ? Math.sqrt(g * x * x / denom) : 0;
//
//        // Vector Compensation
//        Vector robotVelocity = follower.getVelocity();
//        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
//        double parallelComponent = Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        double vz = flywheelSpeed * Math.sin(physicsAngleRad);
//        double time = (flywheelSpeed > 0) ? x / (flywheelSpeed * Math.cos(physicsAngleRad)) : 0.001;
//        double ivr = x / time + parallelComponent;
//        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
//
//        // Recalculate physics angle based on movement
//        physicsAngleRad = Math.atan(vz / nvr);
//
//        // 3. MAP TO YOUR HARDWARE (90 is vertical, 0 is back)
//        // If physics wants 70 degrees, your motor goes to 90 - 70 = 20.
//        double myHardwareHood = 90.0 - Math.toDegrees(physicsAngleRad);
//
//        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);
//        double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);
//
//        CalculationResult res = new CalculationResult();
//        res.ticks = shooterLogic.calcFlywheelSpeedTicks(flywheelSpeed * VECTOR_TUNE_FACTOR);
//
//        // FINAL CLAMP: Only clamp at the very end for your motor limits
//        res.hood = Range.clip(myHardwareHood, 5, 45);
//        res.turret = wrapAngle(turretAngle);
//        return res;
//    }
//
//    private CalculationResult calculateQuarticMath(Pose robotPose) {
//        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
//        double x = Math.hypot(targetX - robotPose.getX(), GOAL_Y - robotPose.getY());
//
//        double velo = (v_a * Math.pow(x, 4)) + (v_b * Math.pow(x, 3)) +
//                (v_c * Math.pow(x, 2)) + (v_d * x) + v_e;
//
//        double hoodPos = (h_a * Math.pow(x, 4)) + (h_b * Math.pow(x, 3)) +
//                (h_c * Math.pow(x, 2)) + (h_d * x) + h_e;
//
//        CalculationResult res = new CalculationResult();
//        res.ticks = velo;
//        res.hood = hoodPos;
//
//        // Basic Auto-Align logic from your setTargetsByDistance
//        double deltaY = GOAL_Y - robotPose.getY();
//        double deltaX = targetX - robotPose.getX();
//        double targetFieldYaw = Math.toDegrees(Math.atan2(-deltaY, deltaX));
//        if (targetFieldYaw < 0) targetFieldYaw += 360;
//
//        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
//        // Normalize 0-360 heading logic like your shooter.setTurretTarget
//        double normalizedHeading = ((-robotHeadingDeg % 360) + 360) % 360;
//        double absoluteTarget = (targetFieldYaw - normalizedHeading + 360) % 360;
//
//        // Map to turret space
//        if (absoluteTarget <= 180) res.turret = absoluteTarget;
//        else res.turret = absoluteTarget - 360;
//
//        return res;
//    }
//
//    private void configurePinpoint() {
//        pinpoint.setOffsets(136.603, 59.24, DistanceUnit.MM);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        pinpoint.resetPosAndIMU();
//    }
//
//    private static class CalculationResult {
//        double ticks, hood, turret;
//    }
//
//    public double wrapAngle(double angle) {
//        // While the angle is greater than 180, subtract a full circle
//        while (angle > 180) {
//            angle -= 360;
//        }
//        // While the angle is less than -180, add a full circle
//        while (angle < -180) {
//            angle += 360;
//        }
//        return angle;
//    }
//}

package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;



import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;



import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

//@Configurable
//@TeleOp(name = "ShootingWhileMovingFull", group = "TEST")
//public class ShootingWhileMovingTester extends OpMode {
//
//    // Field constants
//    public static double RED_GOAL_X = 127.0;
//    public static double BLUE_GOAL_X = 17.0;
//    public static double GOAL_Y = 130.5;
//    public static double VECTOR_TUNE_FACTOR = 2.54;
//    public static double a = -18.35;
//
//    // Shooter tuning
//    public static double HOOD_RATE_DPS = 220;      // hood deg/sec max
//    public static double VELO_RATE_TPS = 12000;    // max tick/sec change while moving
//    public static double TURRET_SMOOTHING = 0.6;  // 0-1, 1 = raw
//
//    private ElapsedTime loopTimer = new ElapsedTime();
//
//    // Subsystems
//    private GamepadEx player1;
//    private MecanumDrive drive;
//    private Follower follower;
//    private GoBildaPinpointDriver pinpoint;
//    private Shooter shooterLogic;
//    private Sensors sensors;
//    private Intake intake;
//    private List<LynxModule> allHubs;
//
//    private boolean enableShooter = false;
//    private boolean isRedAlliance = true;
//
//    // Shooter tracking state
//    private double lastHoodCmd;
//    private double lastVeloCmd;
//    private double lastTurretCmd;
//    private double lastTime;
//    private boolean firstSpinupDone = false;
//
//    private static final double G_INCHES = 386.088;
//
//    @Override
//    public void init() {
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(Poses.getStartingPose());
//
//        drive = new MecanumDrive();
//        drive.init(hardwareMap);
//
//        intake = new Intake(hardwareMap, telemetry);
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        configurePinpoint();
//
//        shooterLogic = new Shooter(hardwareMap, telemetry);
//        sensors = new Sensors();
//        sensors.init(hardwareMap, "SRSHub");
//
//        player1 = new GamepadEx(gamepad1);
//
//        if (Poses.getAlliance() != null)
//            isRedAlliance = (Poses.getAlliance() == Poses.Alliance.RED);
//
//        lastHoodCmd = shooterLogic.getCurrentRequiredHoodAngle();
//        lastVeloCmd = shooterLogic.getCurrentRequiredFlywheelTicks();
//        lastTurretCmd = shooterLogic.getCurrentTurretPosition();
//        lastTime = getRuntime();
//    }
//
//    @Override
//    public void loop() {
//        double now = getRuntime();
//        double dt = Math.max(now - lastTime, 0.001);
//        lastTime = now;
//
//        for (LynxModule hub : allHubs) hub.clearBulkCache();
//
//        player1.readButtons();
//        pinpoint.update();
//        follower.update();
//        sensors.update();
//        shooterLogic.update(sensors.getFlywheelVelo(), shooterLogic.getCurrentTurretPosition());
//
//        intake.update(shooterLogic.isBeamBroken(), LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE, sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
//                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
//                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));
//
//        // ----- INTAKE LOGIC -----
//        if (gamepad1.right_trigger > 0.2) {
//            intake.doTestShooter();
//        } else {
//            intake.doIntakeHalt();
//        }
//
//        // ----- DRIVE (field-relative) -----
//        double forward = player1.getLeftY();
//        double strafe  = player1.getLeftX();
//        double rotate  = player1.getRightX();
//        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }
//
//        double heading = follower.getPose().getHeading();
//        double theta = Math.atan2(forward, strafe) - heading;
//        double r = Math.hypot(forward, strafe);
//        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
//
//        // ----- RELOCALIZE BUTTON -----
//        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
//            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
//        }
//
//        // ----- SHOOTER ENABLE -----
//        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
//            enableShooter = !enableShooter;
//            firstSpinupDone = false; // reset every toggle
//        }
//
//        if (!enableShooter) {
//            shooterLogic.stopFlywheel();
//            shooterLogic.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC, heading, 0);
//            lastTurretCmd = 0;
//            return;
//        }
//
//        // ----- CALCULATE DESIRED TARGETS -----
//        CalculationResult res = calculateShotVector(heading);
//
//        // ----- HOOD RATE LIMITING -----
//        double maxHoodStep = HOOD_RATE_DPS * dt;
//        lastHoodCmd += Range.clip(res.hood - lastHoodCmd, -maxHoodStep, maxHoodStep);
//        shooterLogic.setHoodTargetAngle(lastHoodCmd);
//
//        // ----- FLYWHEEL DYNAMIC RATE LIMITING -----
//        if (!firstSpinupDone) {
//            // instant spin-up on enable
//            lastVeloCmd = res.ticks;
//            firstSpinupDone = true;
//        } else {
//            double maxVeloStep = VELO_RATE_TPS * dt;
//            lastVeloCmd += Range.clip(res.ticks - lastVeloCmd, -maxVeloStep, maxVeloStep);
//        }
//        shooterLogic.setTargetVelocityTicks(lastVeloCmd);
//
//        // ----- TURRET SMOOTHING -----
//        lastTurretCmd += TURRET_SMOOTHING * wrapAngle(res.turret - lastTurretCmd);
//        lastTurretCmd = wrapAngle(lastTurretCmd);
//        shooterLogic.setTurretTargetPosition(lastTurretCmd);
//
//        // ----- TELEMETRY -----
//        telemetry.addData("Hood Target", res.hood);
//        telemetry.addData("Hood Cmd", lastHoodCmd);
//        telemetry.addData("Velo Target", res.ticks);
//        telemetry.addData("Velo Cmd", lastVeloCmd);
//        telemetry.addData("Turret Target", res.turret);
//        telemetry.addData("Turret Cmd", lastTurretCmd);
//        telemetry.addData("Forward", forward);
//        telemetry.addData("Strafe", strafe);
//        telemetry.update();
//    }
//
//    private CalculationResult calculateShotVector(double robotHeading) {
//        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
//
//        Vector robotToGoal = new Vector();
//        robotToGoal.setOrthogonalComponents(
//                targetX - follower.getPose().getX(),
//                GOAL_Y - follower.getPose().getY()
//        );
//
//        double x = Math.max(robotToGoal.getMagnitude() - 5, 1.0);
//        double y = 25.75;
//
//        double physicsAngle = Math.atan(2 * y / x - Math.tan(Math.toRadians(a)));
//        physicsAngle = MathFunctions.clamp(physicsAngle, Math.toRadians(10), Math.toRadians(85));
//
//        double cosA = Math.cos(physicsAngle);
//        double denom = 2 * cosA * cosA * (x * Math.tan(physicsAngle) - y);
//        double flywheelSpeed = (denom > 0) ? Math.sqrt(G_INCHES * x * x / denom) : 0;
//
//        Vector vel = follower.getVelocity();
//        double turretOffset = 0;
//
//        if (vel.getMagnitude() > 1.5) { // only compensate if moving
//            double rel = vel.getTheta() - robotToGoal.getTheta();
//            double parallel = Math.cos(rel) * vel.getMagnitude();
//            double perp     = Math.sin(rel) * vel.getMagnitude();
//
//            double time = x / (flywheelSpeed * cosA);
//            double ivr = x / time + parallel;
//
//            physicsAngle = Math.atan(
//                    (flywheelSpeed * Math.sin(physicsAngle)) /
//                            Math.sqrt(ivr * ivr + perp * perp)
//            );
//
//            turretOffset = Math.atan2(perp, ivr);
//        }
//
//        CalculationResult res = new CalculationResult();
//        res.ticks  = shooterLogic.calcFlywheelSpeedTicks(flywheelSpeed * VECTOR_TUNE_FACTOR);
//        res.hood   = Range.clip(90 - Math.toDegrees(physicsAngle), 5, 45);
//        res.turret = wrapAngle(Math.toDegrees(robotHeading - robotToGoal.getTheta() + turretOffset));
//        return res;
//    }
//
//    private void configurePinpoint() {
//        pinpoint.setOffsets(136.603, 59.24, DistanceUnit.MM);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED
//        );
//        pinpoint.resetPosAndIMU();
//    }
//
//    private static class CalculationResult { double ticks, hood, turret; }
//
//    private double wrapAngle(double a) {
//        while (a > 180) a -= 360;
//        while (a < -180) a += 360;
//        return a;
//    }
//}




import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;



import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "Test Shooter With Motion Compensation", group = "TEST")
public class ShootingWhileMovingTester extends OpMode {

    // --- CONFIGURABLES ---
    public static double RED_GOAL_X = 127.0;
    public static double BLUE_GOAL_X = 17.0;
    public static double GOAL_Y = 130.5;

    public static double VECTOR_TUNE_FACTOR = 2.54;

    // Quartic regression for flywheel & hood (distance → velocity / hood)
    public static double v_a = -0.000419907, v_b = 0.162408, v_c = -21.9297, v_d = 1991.28761, v_e = 150460.825;
    public static double h_a = -(2.52835e-7), h_b = 0.00012437, h_c = -0.0230779, h_d = 1.9269, h_e = -17.60642;

    public static double t_a = -(2.52835e-7), t_b = 0.00012437, t_c = -0.0230779, t_d = 1.9269, t_e = -17.60642;

    public static double a = -18.35; // entry angle for physics solver
    public static double FLYWHEEL_COMP_TUNING = 1.0; // scale velocity compensation

    // --- SYSTEMS ---
    private GamepadEx player1;
    private MecanumDrive drive;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> allHubs;
    private Shooter shooterLogic;
    private Sensors sensors;
    private Intake intake;

    private boolean enableShooter = false;
    private boolean isRedAlliance = true;

    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        player1 = new GamepadEx(gamepad1);
        shooterLogic = new Shooter(hardwareMap, telemetry);
        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        if (Poses.getAlliance() != null) isRedAlliance = (Poses.getAlliance() == Poses.Alliance.RED);
    }

    @Override
    public void loop() {
        // --- Update hardware caches & controls ---
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        player1.readButtons();
        pinpoint.update();
        follower.update();
        sensors.update();
        shooterLogic.update(sensors.getFlywheelVelo(), shooterLogic.getCurrentTurretPosition());

        // --- Intake update ---
        intake.update(
                shooterLogic.isBeamBroken(),
                LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green())
        );

        // --- Relocalization button ---
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

        // --- Shooter enable toggle ---
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) enableShooter = !enableShooter;

        // --- Intake test/shoot ---
        if (gamepad1.right_trigger > 0.2) intake.doTestShooter();
        else intake.doIntakeHalt();

        // --- Drive logic (field-centric) ---
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();
        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }

        double heading = follower.getPose().getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);


        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );

        // --- Shooter logic ---
        if (enableShooter) {
            Pose robotPose = follower.getPose();
            Vector robotVel = follower.getVelocity();
            double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

            calculateNewGoalPos(follower.getPose().getX(),follower.getPose().getY(),RED_GOAL_X,GOAL_Y,robotHeadingDeg,450,0,0,true);

//            setTargetsWithMotionCompensation(
//                    robotPose.getX(), robotPose.getY(),
//                    isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X, GOAL_Y,
//                    robotHeadingDeg,
//                    robotVel,
//                    true, 0, 0, 0, isRedAlliance
//            );
        } else {
            shooterLogic.stopFlywheel();
            shooterLogic.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC, Math.toDegrees(follower.getPose().getHeading()), 0);
        }

        // --- Telemetry ---
        telemetry.addData("Shooter Enabled", enableShooter);
        telemetry.addData("Position", followerData);
        telemetry.addData("Robot Vel Follower", follower.getVelocity().getMagnitude());
        telemetry.addData("Robot X/Y Velo Follower (INCH/SEC)", "%.1f / %.1f", follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Robot X/Y Velo Pinpoint (INCH/SEC)", "%.1f / %.1f", pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelX(DistanceUnit.INCH));
        telemetry.update();

        loopTimer.reset();
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(136.603, 59.24, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    // --- Motion compensation with turret offset ---
    public void setTargetsWithMotionCompensation(
            double robotX, double robotY,
            double goalX, double goalY,
            double robotHeadingDeg,
            Vector robotVelocity,
            boolean autoAlign,
            double flywheelManualAdjust,
            double hoodManualAdjust,
            double turretManualAdjust,
            boolean isRed
    ) {
        // 1️⃣ Distance to goal
        double x = Math.hypot(goalX - robotX, goalY - robotY);

        // 2️⃣ Base hood and flywheel from quartic
        double baseVelo = (v_a * Math.pow(x, 4)) + (v_b * Math.pow(x, 3)) +
                (v_c * Math.pow(x, 2)) + (v_d * x) + v_e;

        double hoodPos = (h_a * Math.pow(x, 4)) + (h_b * Math.pow(x, 3)) +
                (h_c * Math.pow(x, 2)) + (h_d * x) + h_e;

        // 3️⃣ Goal vector (robot → goal)
        Vector goalVector = new Vector();
        goalVector.setOrthogonalComponents(goalX - robotX, goalY - robotY);
        Vector goalUnit = goalVector.normalize();

        // 4️⃣ Motion compensation
        double vParallel = robotVelocity.dot(goalUnit);   // along goal → flywheel
        double vPerp     = robotVelocity.cross(goalUnit); // perpendicular → turret offset

        double compensatedVeloInches = baseVelo + vParallel * FLYWHEEL_COMP_TUNING;
        double compensatedVeloTicks  = shooterLogic.calcFlywheelSpeedTicks(compensatedVeloInches) + flywheelManualAdjust;

        double turretOffsetRad = Math.atan2(vPerp, compensatedVeloInches); // turret adjustment for lateral motion

        // 5️⃣ Base turret angle (auto-align to goal)
        double requiredYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY, isRed);
        double turretAngle = requiredYaw + Math.toDegrees(turretOffsetRad) + turretManualAdjust;

        // 6️⃣ Set shooter targets
        shooterLogic.setTargetVelocityTicks(compensatedVeloTicks);
        shooterLogic.setHoodTargetAngle(Range.clip(hoodPos + hoodManualAdjust, 0, 45));
        if (autoAlign) shooterLogic.setTurretTarget(turretAngle, Shooter.TurretMode.AUTO_ALIGN, robotHeadingDeg, 0);
    }


    public void calculateNewGoalPos( double robotX, double robotY,
                                     double goalX, double goalY,
                                     double robotHeadingDeg, double flywheelOffset,double turretOffset, double hoodOffset, boolean isRed){

        double dist = Math.hypot(goalX - robotX, goalY - robotY);

        double shotTime = (t_a * Math.pow(dist, 4)) + (t_b * Math.pow(dist, 3)) +
                (t_c * Math.pow(dist, 2)) + (t_d * dist) + t_e;

        double newGoalX = goalX - (shotTime * pinpoint.getVelX(DistanceUnit.INCH));
        double newGoalY = goalY - (shotTime * pinpoint.getVelY(DistanceUnit.INCH));

        double newDist = Math.hypot(newGoalX - robotX, newGoalY - robotY);

        double velo = (v_a * Math.pow(newDist, 4)) + (v_b * Math.pow(newDist, 3)) +
                (v_c * Math.pow(newDist, 2)) + (v_d * newDist) + v_e;

        double hoodPos = (h_a * Math.pow(newDist, 4)) + (h_b * Math.pow(newDist, 3)) +
                (h_c * Math.pow(newDist, 2)) + (h_d * newDist) + h_e;

        shooterLogic.setTargetVelocityTicks(velo + flywheelOffset);
        shooterLogic.setHoodTargetAngle(Range.clip(hoodPos + hoodOffset, 0, 45));

        double requiredFieldYaw;
        if (!isRed) {
            requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY, false);
        }
        else { requiredFieldYaw = calculateAutoAlignYaw(robotX, robotY, goalX, goalY, true);}
        // B. Pass to the turret setter
        shooterLogic.setTurretTarget(requiredFieldYaw, Shooter.TurretMode.AUTO_ALIGN, robotHeadingDeg,turretOffset);
    }


    private  double calculateAutoAlignYaw(double robotXInches, double robotYInches,
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
            return targetFieldYawDeg;
        }
        return targetFieldYawDegBlue; // Returns (-180 to 180)
    }

}