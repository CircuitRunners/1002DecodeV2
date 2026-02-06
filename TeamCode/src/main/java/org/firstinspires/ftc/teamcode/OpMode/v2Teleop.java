package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.Config.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Config.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Zenith Teleop", group = "A")
@Configurable
public class v2Teleop extends OpMode {

    private List<LynxModule> allHubs;

    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private LimelightCamera limelight;
    private Sensors sensors;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private GamepadEx player1;
    private GamepadEx player2;

    private int opState = 0;
    private boolean isRedAlliance = true;
    private boolean preselectFromAuto = false;

    private int ballsShotInState = 0;
    private boolean lastBeamState = false;

    private final double RED_GOAL_X = 128;
    private final double BLUE_GOAL_X = 15;
    private final double GOAL_Y = 132;
    private static final double METERS_TO_INCH = 39.37;


    boolean veloReached = false;

    private boolean vibratedYet = false;
    private boolean initiateTransfer = false;

    private boolean noAutoAlign = false;

    private final ElapsedTime timer = new ElapsedTime();
    private double turretMannualAdjust = 0;

    boolean teleopShootApporval = false;

    public static  double[] turretCoefficientsTeleop = {0.12, 0.00, 0.003, 0.003};

    public static double turretDeadband = 0;




    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());
        follower.update();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry,false);
        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();

        sensors = new Sensors();
        sensors.init(hardwareMap, "SRSHub");

        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);

        if(Poses.getAlliance() !=null){
            if(Poses.getAlliance() == Poses.Alliance.RED){
                isRedAlliance = true;
                preselectFromAuto = true;
            }
            else {
                isRedAlliance = false;
                preselectFromAuto = true;
            }
        }
        else{
            isRedAlliance = false;
            preselectFromAuto = false;
        }
    }

    @Override
    public void init_loop() {
        handleAllianceToggles();
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Preselect", preselectFromAuto ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) hub.clearBulkCache();
        timer.reset();
        // --- 1. HARDWARE UPDATES ---
        follower.update();
        pinpoint.update();
        sensors.update();
        player1.readButtons();
        player2.readButtons();

        follower.getVelocity();

        veloReached =  (Math.abs(shooter.getFlywheelVelo()) > (Math.abs(shooter.getTargetFLywheelVelo()) - (1500 )) && Math.abs(shooter.getFlywheelVelo()) < (Math.abs(shooter.getTargetFLywheelVelo()) + (16000)) && Math.abs(shooter.getTargetFLywheelVelo()) >=1);

        shooter.turretCoefficientsTeleop = turretCoefficientsTeleop;
        shooter.turretDeadband = turretDeadband;


        // --- 2. DATA SNAPSHOTS (Call once, reference variables) ---
        Pose currentPose = follower.getPose();
        Pose2D currentPinpointPose = pinpoint.getPosition();
        double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
        double robotVelX = pinpoint.getVelX(DistanceUnit.METER);
        double robotVelY = pinpoint.getVelY(DistanceUnit.METER);

        LimelightCamera.BallOrder activePattern = (Intake.targetPatternFromAuto != null)
                ? Intake.targetPatternFromAuto
                : LimelightCamera.BallOrder.PURPLE_PURPLE_GREEN;

        double currentFlywheelVelo = shooter.getFlywheelVelo();
        double currentTurretAngle = shooter.getCurrentTurretPosition();
        boolean isBeamBroken = shooter.isBeamBroken();


        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPinpointPose.getX(DistanceUnit.INCH),
                currentPinpointPose.getY(DistanceUnit.INCH),
                currentPinpointPose.getHeading(AngleUnit.DEGREES)
        );

        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );

        telemetry.addData("Position", followerData);
        telemetry.addData("Pinpoint (BAD) Position", data);
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("MODE", opState == 0 ? "INTAKE" : opState == 1 ? "BURST" : "SORT");
        telemetry.addData("Loop Time", "%.2f ms", timer.milliseconds());
        telemetry.addData("Flywheel Reached",veloReached );
        telemetry.addData("Turret Reached",shooter.turretReached ? "YEA": "NAH");

        telemetry.addLine("--- DIAGNOSTICS ---");
        telemetry.addData("Turret Ang", "%.2f", currentTurretAngle);
        telemetry.addData("DESIRED VELO:",shooter.getTargetFLywheelVelo());
        telemetry.addData("Flywheel Velo", currentFlywheelVelo);
        telemetry.addData("Beam Broken", isBeamBroken);
        telemetry.addData("Balls shot:", ballsShotInState);


       // telemetry.addData("Shot Possible", !shooter.isShotImpossible);



        // --- 3. LOGIC & OVERRIDES ---
        handleManualTurretOverrides(follower.getPose().getHeading());
        handleAllianceToggles();
        handleInputOverrides();
        handleDriving(currentPose);

        // --- 4. STATE MACHINE ---
        switch (opState) {
            case 0: handleIntakeState(); break;
            case 1: handleScoringStateNoSort(currentPose, robotVelX, robotVelY, currentHeadingDeg, isBeamBroken); break;
            case 2: handleScoringState(currentPose, robotVelX, robotVelY, currentHeadingDeg, isBeamBroken); break;
        }

        // --- 5. SUBSYSTEM UPDATES ---
        intake.setCanShoot(veloReached && shooter.turretReached && teleopShootApporval );
        shooter.update(currentTurretAngle);
        intake.update(isBeamBroken, activePattern,
                sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(), sensors.getColor1Green()),
                sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(), sensors.getColor2Green()),
                sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(), sensors.getColor3Green()));

        // --- 6. TELEMETRY ---

        telemetry.update();

    }

    private void handleManualTurretOverrides(double currentAngle) {
        // Manual control: move turret and stick PID to current position to prevent fighting
        if (gamepad1.dpad_right) {
            turretMannualAdjust +=5;
        }
        else if (gamepad1.dpad_left) {
            turretMannualAdjust -=5;
        }

        // Hardware re-zero
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.rezeroTurretPosition();
            gamepad2.rumble(500);
            //shooter.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC,follower.getPose().getHeading());
        }

        if (player2.wasJustPressed(GamepadKeys.Button.CROSS)){
            shooter.setTurretTarget(limelight.updateError(), Shooter.TurretMode.ROBOT_CENTRIC,currentAngle,0);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
           shooter.resetTurretPID();
        }

        //square turns it off
    }

    private void handleScoringStateNoSort(Pose pose, double vx, double vy, double head, boolean beam) {
        boolean useAprilTagTurret = false;
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            useAprilTagTurret = !useAprilTagTurret;
        }
        if (useAprilTagTurret) {
            noAutoAlign = true;
            updateTurretWithAprilTag();
        } else {
            noAutoAlign = false;
        }

        applyShooterTargets(pose, vx, vy, head);

        if (veloReached  && !vibratedYet) {
            gamepad1.rumble(250);
            vibratedYet = true;
        }
        else if (vibratedYet && (gamepad1.right_trigger > 0.2)){
            initiateTransfer = true;
        }



        if (initiateTransfer){
            trackShotCount(beam);
        }
        if (initiateTransfer && veloReached && (pose.getY() > 80 )){
            teleopShootApporval = true;
            intake.doTestShooter();
        }
        else if (initiateTransfer && veloReached && (pose.getY() <= 80 && gamepad1.right_trigger > 0.2)){
            teleopShootApporval = true;
            intake.doTestShooter();
        }
//        else if (initiateTransfer && !veloReached || (pose.getY() > 80 && gamepad1.right_trigger <=0.17)){
//            intake.doIntakeHalt();
//        }
        else{
            intake.doIntakeHalt();
        }

        //if (ballsShotInState >= 3) resetToIntake();
    }

    private void handleScoringState(Pose pose, double vx, double vy, double head, boolean beam) {
        applyShooterTargets(pose, vx, vy, head);


        if (veloReached  && !vibratedYet) {
            gamepad1.rumble(250);
            vibratedYet = true;
        }
        else if (!intake.canShoot){
            vibratedYet = false;
        }
        if (vibratedYet && (gamepad1.right_trigger > 0.2)){
            initiateTransfer = true;
        }

        if (initiateTransfer){
            teleopShootApporval = true;
            trackShotCount(beam);
        }

        if (ballsShotInState >= 3) resetToIntake();
    }

    private void applyShooterTargets(Pose pose, double vx, double vy, double headingDeg) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
       // shooter.setShooterTarget(pose.getX(), pose.getY(), targetX, GOAL_Y, vx, vy, headingDeg, false); // TRUE for auto align

      if (pose.getY() > 69) {
          if (isRedAlliance) {
              if (noAutoAlign) {
                  shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, false, 0, 0, true, turretMannualAdjust);
              } else {
                  shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, true, 0, 0, true, turretMannualAdjust);
              }
          } else {
              if (noAutoAlign) {
                  shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, false, 0, 0, false, turretMannualAdjust);
              } else {
                  shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, true, 0, 0, false, turretMannualAdjust);
              }
          }
      }

      else {
              if (isRedAlliance) {
                  if (noAutoAlign) {
                      shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, false,0, 0,true,turretMannualAdjust);
                  } else {
                      shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, true, 0,0,true,turretMannualAdjust);
                  }
              }
              else{
                  if (noAutoAlign) {
                      shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, false, 0,0,false,turretMannualAdjust);
                  } else {
                      shooter.setTargetsByDistanceAdjustable(Math.round((pose.getX() * 10) / 10), Math.round((pose.getY() * 10) / 10), targetX, GOAL_Y, headingDeg, true, 0,0,false,turretMannualAdjust);
                  }
              }
          }

    }

    private void handleDriving(Pose pose) {
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        if (!isRedAlliance) { forward = -forward; strafe = -strafe; }
        double heading = pose.getHeading();
        double theta = Math.atan2(forward, strafe) - heading;
        double r = Math.hypot(forward, strafe);
        drive.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void trackShotCount(boolean currentBeamState) {
        if (lastBeamState && !currentBeamState) {
            ballsShotInState++;
        }
        lastBeamState = currentBeamState;
    }

    private void resetToIntake() {
        opState = 0;
        ballsShotInState = 0;
        teleopShootApporval = false;
        initiateTransfer = false;
        vibratedYet = false;
        shooter.stopFlywheel();
        //intake.sortManualOverride();
        intake.resetState();
       gamepad1.rumble(150);
    }

    private void handleAllianceToggles() {

        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) {
            if (isRedAlliance){
            isRedAlliance = false; gamepad1.rumble(100); }
            else {
                isRedAlliance = true; gamepad1.rumble(100);
            }
        }
    }

    private void handleInputOverrides() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) updateCoordinatesWithAprilTag();
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            LimelightCamera.BallOrder seen = limelight.detectBallOrder();
            if (seen != null) { Intake.targetPatternFromAuto = seen; gamepad1.rumble(500); }
        }
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (opState ==0) { ballsShotInState = 0; opState = 1; } else resetToIntake();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
            noAutoAlign = true;
        }
        /* not till tuned sry lil bro
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (opState == 0) { ballsShotInState = 0; opState = 2;
             intake.prepareAndStartSort();
             }
             else {
             resetToIntake();
             }
        }
        */

    }

    private void handleIntakeState() {

        //shooter.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC,follower.getPose().getHeading());
        if (gamepad1.right_trigger > 0.2){
            intake.doIntake();
            if (!noAutoAlign){
            shooter.setTurretTarget(0, Shooter.TurretMode.ROBOT_CENTRIC, follower.getPose().getHeading(), turretMannualAdjust);
            }

        }
        else if (gamepad1.left_trigger > 0.2) intake.doOuttake();
        else intake.doIntakeHalt();
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(136.603, 59.24, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

//    private void doTelemetry() {
//
//        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
//        telemetry.addData("MODE", opState == 0 ? "INTAKE" : opState == 1 ? "BURST" : "SORT");
//        telemetry.addData("Loop Time", "%.2f ms", timer.milliseconds());
//        telemetry.addData("Flywheel Reached",shooter.flywheelVeloReached ? "YEA": "NAH");
//        telemetry.addData("Turret Reached",shooter.turretReached ? "YEA": "NAH");
//        telemetry.update();
//    }

    private void extraTelemetryForTesting(double fVelo, double tAng, boolean beam) {
        telemetry.addLine("--- DIAGNOSTICS ---");
        telemetry.addData("Turret Ang", "%.2f", tAng);
        telemetry.addData("Flywheel Velo", fVelo);
        telemetry.addData("Beam Broken", beam);
        //telemetry.addData("Shot Possible", !shooter.isShotImpossible);
        telemetry.update();
    }

    public void updateCoordinatesWithAprilTag() {
        limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();
        if (result != null && result.isValid()) {
            Pose3D mt1Pose = result.getBotpose();
            if (mt1Pose != null) {
                double finalX = (mt1Pose.getPosition().y * METERS_TO_INCH) + 72.0;
                double finalY = (-mt1Pose.getPosition().x * METERS_TO_INCH) + 72.0;
                follower.setPose(new Pose(finalX, finalY, follower.getHeading()));
                gamepad1.rumble(500);
            }
        }
    }

    public void updateTurretWithAprilTag() {
        if (follower.getPose().getY() > 72.0) {
            limelight.limelightCamera.pipelineSwitch(5);
            LLResult result = limelight.getResult();
            if (result != null && result.isValid()) {
                double error = limelight.updateError();
                double currentTurretAngle = shooter.getCurrentTurretPosition();
                double newTarget = currentTurretAngle + error;
                if (newTarget < 0) newTarget += 360;
                shooter.setTurretTargetPosition(newTarget);
                gamepad1.rumble(200);

            }
        }

    }

    @Override public void stop() { limelight.limelightCamera.stop(); }

//    public void calculateIterativeShot(){
//        double x = Math.hypot(isRedAlliance?RED_GOAL_X:BLUE_GOAL_X - follower.getPose().getX(), GOAL_Y - follower.getPose().getY());
//        Vector robotVelocity = new Vector(follower.getPose());
//        Vector robotToGoalVector = new Vector(isRedAlliance ? RED_GOAL_X: BLUE_GOAL_X, GOAL_Y);
//
//        double coordinateTheta =
//                robotVelocity.getTheta() - robotToGoalVector.getTheta();
//
//        double parallelComponent =
//                -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//
//        double perpendicularComponent =
//                -Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//// velocity compensation variables
//        double vz = shooter.calcFlywheelSpeedInches(shooter.getCurrentRequiredFlywheelTicks()) * Math.sin(shooter.getCurrentRequiredHoodAngle());
//        double time = x / (shooter.calcFlywheelSpeedInches(shooter.getCurrentRequiredFlywheelTicks())) * Math.cos(shooter.getCurrentRequiredHoodAngle());
//        double ivr = x / time + parallelComponent;
//        double nvr = Math.sqrt(
//                ivr * ivr + perpendicularComponent * perpendicularComponent
//        );
//        double ndr = nvr * time;
//    }
}