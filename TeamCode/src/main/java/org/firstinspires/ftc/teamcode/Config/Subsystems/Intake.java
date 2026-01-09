package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;

@Configurable
public class Intake {

    private Telemetry telemetry;
    private DcMotorEx intake;
    private Servo transferDirectionSwitcher;
    private Servo transferPowerTransmition;
    private Servo gateLeft;
    private Servo gateRight;

    public static double motorPower = 0;

    // Configurable constants
    public static final int TICKS_PER_REV = 537; // goBILDA 312 RPM Yellow Jacket
    public static double targetRPM = 0;  // default target speed
    public static final double TRANSFER_DIRECTION_TRANSFER_POS = 0.15;
    public static final double TRANSFER_DIRECTION_CYCLE_POS = 0.42;
    public static final double GATE_OPEN = 0.15;
    public static final double GATE_CLOSED = 0.77;
    public static final double TRANSFER_ON = 0.5;
    public static final double TRANSFER_OFF = 0.32;

    //SORTING STUFF//
    private int currentShot = 0;
    private int greenInventory = 0;
    private int purpleInventory = 0;
    private boolean ballInTransfer = false;
    private boolean greenHasBeenShot = false;
    private boolean patternReady = false;

    public static boolean canShoot = false;

    public static LimelightCamera.BallOrder targetPatternFromAuto = null;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transferDirectionSwitcher = hardwareMap.get(Servo.class,"directionSwitch");
        transferDirectionSwitcher.setDirection(Servo.Direction.FORWARD);

        transferPowerTransmition = hardwareMap.get(Servo.class,"centerClutch");
        transferPowerTransmition.setDirection(Servo.Direction.FORWARD);

        gateLeft = hardwareMap.get(Servo.class,"gateLeft");
        gateLeft.setDirection(Servo.Direction.REVERSE);

        gateRight = hardwareMap.get(Servo.class,"gateRight");
        gateRight.setDirection(Servo.Direction.FORWARD);


    }



    public void intake(){
        gateClose();
        transferOff();
        intake.setPower(1);
        motorPower = 1;
    }

    public void outtake(){
        gateClose();
        transferOn();
        setDirectionCycle();
        intake.setPower(-1);
        motorPower = -0.4;
    }

    public void retainBalls(){
        gateClose();
        transferOff();
        intake.setPower(0.5);
        motorPower = 0.5;
    }

    public void intakeMotorIdle(){
        intake.setPower(0);
        motorPower = 0;
    }

    public void setDirectionSwitcherPosition(double position) {
        transferDirectionSwitcher.setPosition(Range.clip(position, 0.0,1.0));
    }

    public void setPowerTransmitionPosition(double position) {
        transferPowerTransmition.setPosition(Range.clip(position, 0.0,1.0));
    }

    public void setGatePositon(double position) {
        gateLeft.setPosition(Range.clip(position, 0.0,1.0));
        gateRight.setPosition(Range.clip(position, 0.0,1.0));
    }

    private void transferOn(){
        setPowerTransmitionPosition(TRANSFER_ON);
    }

    private void transferOff(){
        setPowerTransmitionPosition(TRANSFER_OFF);
    }

    public void setDirectionTransfer(){
        setDirectionSwitcherPosition(TRANSFER_DIRECTION_TRANSFER_POS);
    }

    public void setDirectionCycle(){
        setDirectionSwitcherPosition(TRANSFER_DIRECTION_CYCLE_POS);
    }

    public void gateOpen(){
        setGatePositon(GATE_OPEN);
    }
    public void gateClose(){
        setGatePositon(GATE_CLOSED);
    }

    public void setCanShoot(boolean value){
        canShoot = value;
    }


    public void cycle(){
        gateOpen();
        transferOn();
        setDirectionCycle();
        intake.setPower(0.85);
    }

    public void transfer(){
        transferOn();
        setDirectionTransfer();
        intake.setPower(1);
    }

    public void resetIndexer(){
        transferOff();
        setDirectionCycle();
        intakeMotorIdle();
    }

    public void sortManualOverride() {
        ballInTransfer = false;
        currentShot = 0;

        greenInventory = 0;
        purpleInventory = 0;

        resetIndexer();

        telemetry.addLine("Sorter manual override triggered: sorting stopped.");
    }



//    public void sort(double shooterBeamBrake, LimelightCamera.BallOrder targetOrder,
//                     DetectedColor colorSensor1Value,
//                     DetectedColor colorSensor2Value,
//                     DetectedColor colorSensor3Value) {
//
//        boolean isBeamBroken = (shooterBeamBrake <= 0.5);
//
//
//        String[] fullTargetSequence;
//        if (targetOrder == LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE) {
//            fullTargetSequence = new String[]{"Green", "Purple", "Purple"};
//        } else if (targetOrder == LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE) {
//            fullTargetSequence = new String[]{"Purple", "Green", "Purple"};
//        } else if (targetOrder == null){
//            fullTargetSequence = new String[]{"Purple","Purple","Purple"};
//            greenHasBeenShot = true;
//        }
//        else {
//            fullTargetSequence = new String[]{"Purple", "Purple", "Green"};
//        }
//
//        //  Initialize inventory once at the start
////        if (currentShot == 0 && greenInventory == 0 && purpleInventory == 0) {
////            DetectedColor[] sensors = {colorSensor1Value, colorSensor2Value, colorSensor3Value};
////            for (DetectedColor color : sensors) {
////                if (color == DetectedColor.GREEN) greenInventory++;
////                else if (color == DetectedColor.PURPLE) purpleInventory++;
////            }
////        }
//        purpleInventory = getPurpleInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value);
//        greenInventory = getGreenInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value);
//
//        int totalBalls = getTotalInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value);
//        if (totalBalls == 0) return; // nothing to sort
//
//       boolean allPurple = totalBalls - greenInventory == totalBalls;
//       boolean allGreen = totalBalls - purpleInventory == totalBalls;
//
//        int shotsToTake = Math.min(3, totalBalls);
//
//
//        if (currentShot >= shotsToTake) {
//            telemetry.addLine("Sorting complete. Resetting sorter state.");
//            currentShot = 0;
//            greenInventory = 0;
//            purpleInventory = 0;
//            ballInTransfer = false;
//            greenHasBeenShot = false;
//            return;
//        }
//
//        String requiredColor = fullTargetSequence[currentShot];
//        DetectedColor topBall = colorSensor1Value; // top-most ball sensor
//
//        boolean correctBallAvailable = (requiredColor.equals("Green") && greenInventory > 0)
//                || (requiredColor.equals("Purple") && purpleInventory > 0);
//
//
//        if (allGreen || allPurple){
//            if (!ballInTransfer) {
//                if (canShoot){
//                    transfer();
//                    ballInTransfer = true;
//                }
//            }
//        }
//        else {
//            if (!greenHasBeenShot) {
//                if (!ballInTransfer) {
//                    // Transfer if top ball matches required color or if no correct color remains
//                    if ((requiredColor.equals("Green") && topBall == DetectedColor.GREEN) ||
//                            (requiredColor.equals("Purple") && topBall == DetectedColor.PURPLE) ||
//                            (!correctBallAvailable && topBall != null)) {
//                       if (canShoot){
//                           transfer();
//                           ballInTransfer = true;
//                       }
//
//                    } else {
//                        cycle();
//                    }
//                }
//            } else if (greenHasBeenShot) {
//                if (!ballInTransfer) {
//                    if (canShoot){
//                        transfer();
//                        ballInTransfer = true;
//                    }
//                }
//            }
//        }
//
//
//        if (ballInTransfer && isBeamBroken) {
//            // Update inventory
//            if (requiredColor.equals("Green") && greenInventory > 0) {
//                greenInventory--;
//            }
//            else if (requiredColor.equals("Purple") && purpleInventory > 0) purpleInventory--;
//
//            // Advance slot
//            currentShot++;
//            ballInTransfer = false;
//            if (requiredColor.equals("Green")){
//                greenHasBeenShot = true;
//            }
//        }
//
//        // --- Keep cycling leftover balls until they reach transfer ---
//        if (currentShot < shotsToTake && !ballInTransfer) {
//            cycle();
//        }
//    }

    private boolean lastSlot1WasNull = false;
    private boolean patternIsLocked = false; // New flag to stop checking sensors during firing

    public void sort(double shooterBeamBrake, LimelightCamera.BallOrder targetOrder,
                     DetectedColor colorSensor1Value,
                     DetectedColor colorSensor2Value,
                     DetectedColor colorSensor3Value) {

        boolean isBeamBroken = (shooterBeamBrake <= 0.5);

        // 1. Get current state
        DetectedColor slot1 = colorSensor1Value;
        DetectedColor slot2 = colorSensor2Value;
        DetectedColor slot3 = colorSensor3Value;
        int totalBalls = getTotalInventory(slot1, slot2, slot3);

        // 2. Completion / Reset Logic
        if (totalBalls == 0 || currentShot >= 3) {
//            if (currentShot >= 3) {
                currentShot = 0;
                patternIsLocked = false;
         //   }
            resetIndexer();
            return;
        }

        // 3. Phase 1: Sorting (Only runs if we haven't started firing yet)
        if (!patternIsLocked) {
            if (totalBalls <= 1) {
                patternIsLocked = true; // Nothing to sort, just fire
            } else {
                // Define target pattern
                String[] pattern = getTargetArray(targetOrder);

                boolean s1Match = isColorMatch(slot1, pattern[0]);
                boolean s2Match = isColorMatch(slot2, pattern[1]);

                if (s1Match && s2Match) {
                    gateClose();
                    intakeMotorIdle();
                    patternIsLocked = true; // ORDER ACQUIRED
                } else {
                    performSortingCycle(slot1);
                }
            }
        }

        // 4. Phase 2: Firing (The "Boom Boom Boom" Phase)
        if (patternIsLocked) {
            if (!ballInTransfer) {
                if (canShoot) {
                    transfer();
                    ballInTransfer = true;
                } else {
                    gateClose();
                    intakeMotorIdle();
                }
            }
        }

        // 5. Shot Counter
        if (ballInTransfer && isBeamBroken) {
            currentShot++;
            ballInTransfer = false;
        }
    }

    /**
     * Cycle logic: Opens gate to rotate balls,
     * but closes gate immediately when a new ball enters Slot 1.
     */
    private void performSortingCycle(DetectedColor currentSlot1) {
        boolean isSlot1Null = (currentSlot1 == null);

        // Transition: Slot 1 was empty, now it's not = ball "caught"
        if (lastSlot1WasNull && !isSlot1Null) {
            gateClose();
            intakeMotorIdle();
        } else {
            cycle(); // Opens gate and runs motor
        }

        lastSlot1WasNull = isSlot1Null;
    }

    private String[] getTargetArray(LimelightCamera.BallOrder targetOrder) {
        if (targetOrder == LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE)
            return new String[]{"Green", "Purple", "Purple"};
        if (targetOrder == LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE)
            return new String[]{"Purple", "Green", "Purple"};
        if (targetOrder == null)
            return new String[]{"Purple", "Purple", "Purple"};

        return new String[]{"Purple", "Purple", "Green"};
    }

    private boolean isColorMatch(DetectedColor sensor, String required) {
        if (sensor == null) return false;
        return required.equals("Green") ? (sensor == DetectedColor.GREEN) : (sensor == DetectedColor.PURPLE);
    }

    public int getGreenInventory(DetectedColor colorSensor1Value,
                                 DetectedColor colorSensor2Value,
                                 DetectedColor colorSensor3Value) {
        greenInventory = 0;
        DetectedColor[] sensors = {colorSensor1Value, colorSensor2Value, colorSensor3Value};
        for (DetectedColor color : sensors) {
            if (color == DetectedColor.GREEN) greenInventory++;
        }

        return greenInventory;

    }

    public int getPurpleInventory(DetectedColor colorSensor1Value,
                                 DetectedColor colorSensor2Value,
                                 DetectedColor colorSensor3Value) {
        purpleInventory = 0;
        DetectedColor[] sensors = {colorSensor1Value, colorSensor2Value, colorSensor3Value};
        for (DetectedColor color : sensors) {
            if (color == DetectedColor.PURPLE) purpleInventory++;
        }

        return purpleInventory;

    }

    public int getTotalInventory(DetectedColor colorSensor1Value,
                                 DetectedColor colorSensor2Value,
                                 DetectedColor colorSensor3Value) {
        return getGreenInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value) +
                getPurpleInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value);
    }

    // --- Periodic update (optional) ---
    public void update() {
//        telemetry.addData("Target RPM", targetRPM);
//        telemetry.addData("Current RPM", getCurrentRPM());
        telemetry.addData("Servo Pos", transferDirectionSwitcher.getPosition());
        telemetry.update();
    }

    // --- Utility methods ---


    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    public double getCurrentRPM() {
        return intake.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getCurrentVelocity(){
        return intake.getVelocity();
    }
    public double getCurrentTargetRPM(){
        return targetRPM;
    }

    public double getIntakeMotorCurrent(){
        return intake.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getCurrentTargetVelocity(){
        return rpmToTicksPerSecond(targetRPM);
    }
    public double getPower(){
        return motorPower;
    }






}

