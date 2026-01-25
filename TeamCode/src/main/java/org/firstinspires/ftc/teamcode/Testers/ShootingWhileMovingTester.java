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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "EXACT MATH COMPARISON", group = "TEST")
public class ShootingWhileMovingTester extends OpMode {

    // Target Constants
    public static double RED_GOAL_X = 127.0;
    public static double BLUE_GOAL_X = 17.0;
    public static double GOAL_Y = 129.0;

    public static double VECTOR_TUNE_FACTOR = 2.56;

    // Subsystems
    private GamepadEx player1;
    private MecanumDrive drive;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> allHubs;
    private Shooter shooterLogic;

    private boolean isRedAlliance = true;

    public static double a = -18.35;

    // Quartic Coefficients from your Shooter class
    private final double v_a = -0.000419907, v_b = 0.162408, v_c = -21.9297, v_d = 1991.28761, v_e = 150460.825;
    private final double h_a = -(2.52835e-7), h_b = 0.00012437, h_c = -0.0230779, h_d = 1.9269, h_e = -17.60642;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        player1 = new GamepadEx(gamepad1);
        shooterLogic = new Shooter(hardwareMap, telemetry);

        if(Poses.getAlliance() != null) {
            isRedAlliance = (Poses.getAlliance() == Poses.Alliance.RED);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        player1.readButtons();
        pinpoint.update();
        follower.update();

        // --- DRIVING (v2Teleop Logic) ---
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = follower.getPose().getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);

        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

        // --- RUN CALCULATIONS ---
        CalculationResult vectorRes = calculateShotVectorExact(heading);
        CalculationResult quarticRes = calculateQuarticMath(follower.getPose());

        // --- TELEMETRY ---
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");

        telemetry.addLine("\n=== VECTOR METHOD (EXACT) ===");
        telemetry.addData("V-Ticks", "%.1f", vectorRes.ticks);
        telemetry.addData("V-Hood", "%.2f", vectorRes.hood);
        telemetry.addData("V-Turret", "%.2f", vectorRes.turret);

        telemetry.addLine("\n=== QUARTIC METHOD (EXACT) ===");
        telemetry.addData("Q-Ticks", "%.1f", quarticRes.ticks);
        telemetry.addData("Q-Hood", "%.2f", quarticRes.hood);
        telemetry.addData("Q-Turret", "%.2f", quarticRes.turret);

        telemetry.update();
    }

    /**
     * Re-implemented EXACTLY from your original provided snippet
     */
    private CalculationResult calculateShotVectorExact(double robotHeading) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        Vector robotToGoalVector = new Vector();
        robotToGoalVector.setOrthogonalComponents(targetX - follower.getPose().getX(), GOAL_Y - follower.getPose().getY());

        double g = 386.088;
        double x = Math.max(robotToGoalVector.getMagnitude() - 5, 1.0);
        double y = (38.75 - 16) + 3;

        // START HERE: Use a very simple, neutral entry angle.
        // As you get closer (x small), 2y/x grows, making the angle steeper.
        double aLocal = Math.toRadians(a);

        // 1. Solve Physics Angle (0 is flat, 90 is vertical)
        // IMPORTANT: Allow this to solve for the FULL physical range (10 to 85 degrees)
        double physicsAngleRad = Math.atan(2 * y / x - Math.tan(aLocal));
        physicsAngleRad = MathFunctions.clamp(physicsAngleRad, Math.toRadians(10), Math.toRadians(85));

        // 2. Velocity Solve (Original stable logic)
        double denom = (2 * Math.pow(Math.cos(physicsAngleRad), 2) * (x * Math.tan(physicsAngleRad) - y));
        double flywheelSpeed = (denom > 0) ? Math.sqrt(g * x * x / denom) : 0;

        // Vector Compensation
        Vector robotVelocity = follower.getVelocity();
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
        double parallelComponent = Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(physicsAngleRad);
        double time = (flywheelSpeed > 0) ? x / (flywheelSpeed * Math.cos(physicsAngleRad)) : 0.001;
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);

        // Recalculate physics angle based on movement
        physicsAngleRad = Math.atan(vz / nvr);

        // 3. MAP TO YOUR HARDWARE (90 is vertical, 0 is back)
        // If physics wants 70 degrees, your motor goes to 90 - 70 = 20.
        double myHardwareHood = 90.0 - Math.toDegrees(physicsAngleRad);

        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);
        double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);

        CalculationResult res = new CalculationResult();
        res.ticks = shooterLogic.calcFlywheelSpeedTicks(flywheelSpeed * VECTOR_TUNE_FACTOR);

        // FINAL CLAMP: Only clamp at the very end for your motor limits
        res.hood = Range.clip(myHardwareHood, 5, 45);
        res.turret = wrapAngle(turretAngle);
        return res;
    }

    private CalculationResult calculateQuarticMath(Pose robotPose) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        double x = Math.hypot(targetX - robotPose.getX(), GOAL_Y - robotPose.getY());

        double velo = (v_a * Math.pow(x, 4)) + (v_b * Math.pow(x, 3)) +
                (v_c * Math.pow(x, 2)) + (v_d * x) + v_e;

        double hoodPos = (h_a * Math.pow(x, 4)) + (h_b * Math.pow(x, 3)) +
                (h_c * Math.pow(x, 2)) + (h_d * x) + h_e;

        CalculationResult res = new CalculationResult();
        res.ticks = velo;
        res.hood = hoodPos;

        // Basic Auto-Align logic from your setTargetsByDistance
        double deltaY = GOAL_Y - robotPose.getY();
        double deltaX = targetX - robotPose.getX();
        double targetFieldYaw = Math.toDegrees(Math.atan2(-deltaY, deltaX));
        if (targetFieldYaw < 0) targetFieldYaw += 360;

        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        // Normalize 0-360 heading logic like your shooter.setTurretTarget
        double normalizedHeading = ((-robotHeadingDeg % 360) + 360) % 360;
        double absoluteTarget = (targetFieldYaw - normalizedHeading + 360) % 360;

        // Map to turret space
        if (absoluteTarget <= 180) res.turret = absoluteTarget;
        else res.turret = absoluteTarget - 360;

        return res;
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(136.603, 59.24, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    private static class CalculationResult {
        double ticks, hood, turret;
    }

    public double wrapAngle(double angle) {
        // While the angle is greater than 180, subtract a full circle
        while (angle > 180) {
            angle -= 360;
        }
        // While the angle is less than -180, add a full circle
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}