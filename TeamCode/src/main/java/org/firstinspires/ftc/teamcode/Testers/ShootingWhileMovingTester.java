package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;

import java.util.List;
@Configurable
@TeleOp(name = "SSHOOTING WHILE MOVING TEST", group = "TEST")
public class ShootingWhileMovingTester extends OpMode{




        // ===== Physics Constants (Standardized for 10ft Test) =====
        public static double TEST_DISTANCE_INCHES = 120.0;
        public static double GOAL_HEIGHT = 38.75;
        public static double LAUNCH_HEIGHT = 12.0;
        public static double GRAVITY = 386.1;

    private final double RED_GOAL_X = 132.0;
    private final double BLUE_GOAL_X = 12.0;
    private final double GOAL_Y = 140.0;

        // ===== Dashboard Tunables =====
//        public static double targetTicksPerSec = 200000.0;
//        public static double desiredHoodAngle = 30.0;
        public static boolean enableShooter = false; // Safety toggle

        private Sensors sensors;
        private Shooter shooter;
        private Intake intake;
        private List<LynxModule> allHubs;
    private GoBildaPinpointDriver pinpoint;



        // Debounce state
        private boolean lastButton = false;

        @Override
        public void init() {
            // 1. Setup Hubs for Manual Caching
            allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            // 2. Initialize Subsystems
            sensors = new Sensors();
            sensors.init(hardwareMap, "SRSHub");

            sensors.update();

            // Shooter constructor now handles PID initialization correctly
            shooter = new Shooter(hardwareMap, telemetry);

            shooter.update(shooter.getFlywheelVelo(), shooter.getCurrentTurretPosition());

            intake = new Intake(hardwareMap, telemetry);

            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            configurePinpoint();

            telemetry.addLine("Initialized.");
            telemetry.addLine("Hood will move immediately on START.");
            telemetry.addLine("Press Square/X to toggle Flywheel.");
            telemetry.update();
        }

        @Override
        public void loop() {
            // --- STEP 1: REFRESH HARDWARE DATA ---
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            sensors.update();

            // --- STEP 2: HANDLE INPUTS ---

            // Debounced Toggle for Square (PS) or X (Xbox)
            boolean currentButton = gamepad1.square || gamepad1.x;
            if (currentButton && !lastButton) {
                enableShooter = !enableShooter;
            }
            lastButton = currentButton;

            // Tuning Controls
//            if (gamepad1.dpad_up) targetTicksPerSec += 5000;
//            if (gamepad1.dpad_down) targetTicksPerSec -= 5000;
//            if (gamepad1.right_bumper) desiredHoodAngle += 2;
//            if (gamepad1.left_bumper) desiredHoodAngle -= 2;
//            if (gamepad1.circle){
//                targetTicksPerSec +=1000;
//            }
//            if (gamepad1.triangle){
//                targetTicksPerSec -=1000;
//            }

            if (gamepad1.right_trigger > 0.2){
                intake.transfer();
            }
            else{
                intake.intakeMotorIdle();
            }

            if (gamepad1.left_stick_button) pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72,72,AngleUnit.RADIANS,Math.toRadians(90)));



            // --- STEP 3: APPLY LOGIC TO SUBSYSTEM ---
            double currentVelo = shooter.getFlywheelVelo();
            double currentTurret = shooter.getCurrentTurretPosition();

            // Always update hood angle so it responds even if flywheel is off


            if (enableShooter) {
                shooter.calculateIterativeShot(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), RED_GOAL_X,GOAL_Y,pinpoint.getVelX(DistanceUnit.INCH),pinpoint.getVelY(DistanceUnit.INCH),pinpoint.getHeading(AngleUnit.DEGREES),true);
            } else {
                // This sets the internal target to 0 and cuts power
                shooter.stopFlywheel();
            }

            // Write to hardware (Servos + Motors)
            shooter.update(currentVelo, currentTurret);

            // --- STEP 4: PHYSICS CALCULATIONS (K-FACTOR) ---

            telemetry.addData("Actual Ticks", "%.1f", currentVelo);
            telemetry.addData("Hood Angle",shooter.getCurrentRequiredHoodAngle());
            telemetry.addData("Turret Pos", "%.1f", currentTurret);
            telemetry.update();
        }

    private void configurePinpoint() {
        pinpoint.setOffsets(44.94, -170.367, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }
    }


