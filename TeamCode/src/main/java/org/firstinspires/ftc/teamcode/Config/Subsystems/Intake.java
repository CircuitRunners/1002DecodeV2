package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;

import java.util.Timer;

@Configurable
public class Intake {

    private Telemetry telemetry;
    private DcMotorEx intake;
    private Servo transferDirectionSwitcher;
    private Servo transferPowerTransmition;
    private Servo gateLeft;
    private Servo gateRight;

    public static double motorPower = 0;

    private boolean lastBeamState = false;

    // Configurable constants
    public static final int TICKS_PER_REV = 537; // goBILDA 312 RPM Yellow Jacket
    public static double targetRPM = 0;  // default target speed
    public static final double TRANSFER_DIRECTION_TRANSFER_POS = 0.35;
    public static final double TRANSFER_DIRECTION_CYCLE_POS = 0.43;
    public static final double GATE_OPEN = 0.15;
    public static final double GATE_CLOSED = 0.81;
    public static final double TRANSFER_ON = 0.59;
    public static final double TRANSFER_OFF = 0.67;

    boolean isFirstTime = true;

    //SORTING STUFF//
    public enum IntakeState {
        IDLE,
        COASTING_TO_SLOT,  // Moving ball toward sensors
        POSITION_NUDGE,    // Backwards pulse for "Ball Hole" fix
        VERIFYING_ORDER,   // Checking if S1/S2 match target pattern
        FLUSHING,          // Clearing top ball if order is wrong
        READY_TO_FIRE,     // Pattern locked, waiting for canShoot
        TRANSFERRING,       // Moving ball to shooter
        INTAKING,
        OUTTAKING,
        HALT,
        PRE_NUDGE,
        RESET,
        CYCLE,
        TEST_SHOOTING
    }

    public enum SimpleSortState {
        IDLE,
        CHECK_SLOTS,
        ROTATING,
        READY
    }

    private IntakeState currentState = IntakeState.IDLE;
    public ElapsedTime stateTimer = new ElapsedTime();

    private SimpleSortState simpleSortState = SimpleSortState.IDLE;



    // Logic Latches (The "Memory" of the ball positions)
    private boolean ball1Latched = false;
    private boolean ball2Latched = false;

    // Counters & Inventory
    private int stabilityCounter = 0; // For "No-Bounce" confirmation
    private int nudgeCount = 0;       // Safety to prevent infinite nudging
    private int cycleCount = 0;       // Tracks if we've seen all balls
    private int greenInventory = 0;
    private int purpleInventory = 0;
    private int internalTotalBalls = 0;
    private int currentShot = 0;
    //private boolean ballInTransfer = false;

//    private boolean s1HadBallLast = true;         // previous loop
//    private boolean s1LostAfterHaving = false;     // did the ball leave the slot after having one
//    private double s1LossTime = 0;                 // time when slot became empty
//    private int s1BallEntryCount = 0;              // total number of balls detected
//    private static final double ENTRY_DEBOUNCE_MS = 550; // ignore balls re-entering within 100ms


    public static boolean canShoot = false;

    public static LimelightCamera.BallOrder targetPatternFromAuto = null;



    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transferDirectionSwitcher = hardwareMap.get(Servo.class,"directionSwitch");
        transferDirectionSwitcher.setDirection(Servo.Direction.REVERSE);

        transferPowerTransmition = hardwareMap.get(Servo.class,"centerClutch");
        transferPowerTransmition.setDirection(Servo.Direction.REVERSE);

        gateLeft = hardwareMap.get(Servo.class,"gateLeft");
        gateLeft.setDirection(Servo.Direction.REVERSE);

        gateRight = hardwareMap.get(Servo.class,"gateRight");
        gateRight.setDirection(Servo.Direction.FORWARD);

    }


    public void doIntake(){
        newState(IntakeState.INTAKING);
    }

    public void doOuttake(){
        newState(IntakeState.OUTTAKING);
    }
    public void doIntakeHalt(){
        newState(IntakeState.HALT);
    }
    public void doCycle(){
        newState(IntakeState.CYCLE);
    }
    public void doTransfer(){
        internalTotalBalls = 3;
        newState(IntakeState.TRANSFERRING);
    }
    public void doTestShooter(){
        newState(IntakeState.TEST_SHOOTING);
    }
    public void resetState(){
        newState(IntakeState.RESET);
    }

    private void intake(){
        gateClose();
        transferOff();
        setDirectionCycle();
        intake.setPower(1);
        motorPower = 1;
    }

    private void outtake(){
        gateClose();
        transferOn();
        setDirectionCycle();
        intake.setPower(-1);
        motorPower = -1;
    }

    private void retainBalls(){
        gateClose();
        transferOff();
        intake.setPower(0.75);
        motorPower = 0.5;
    }

    private void intakeMotorHalt(){
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


    private void cycle(){
        gateOpen();
        transferOn();
        setDirectionCycle();
        intake.setPower(0.9);
    }

    private void transfer(){
        transferOn();
        setDirectionTransfer();
        gateClose();
        intake.setPower(1);
    }

    private void resetIndexer(){
        transferOff();
        setDirectionCycle();
        intakeMotorHalt();
    }



//    public void sortManualOverride() {
//        ballInTransfer = false;
//        patternIsLocked = false;
//
//        currentShot = 0;
//
//        greenInventory = 0;
//        purpleInventory = 0;
//
//        resetIndexer();
//
//        telemetry.addLine("Sorter manual override triggered: sorting stopped.");
//    }



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

//    private boolean lastSlot1WasNull = false;
//    private boolean patternIsLocked = false; // New flag to stop checking sensors during firing

//    public void sort(boolean shooterBeamBrake, LimelightCamera.BallOrder targetOrder,
//                     DetectedColor colorSensor1Value,
//                     DetectedColor colorSensor2Value,
//                     DetectedColor colorSensor3Value) {
//
//        boolean isBeamBroken = shooterBeamBrake;
//
//        // 1. Get current state
//        DetectedColor slot1 = colorSensor1Value;
//        DetectedColor slot2 = colorSensor2Value;
//        DetectedColor slot3 = colorSensor3Value;
//        int totalBalls = getTotalInventory(slot1, slot2, slot3);
//
//        // 2. Completion / Reset Logic
//        if (totalBalls == 0 ) {
////            if (currentShot >= 3) {
//                currentShot = 0;
//                patternIsLocked = false;
//                lastSlot1WasNull = true;
//         //   }
//            resetIndexer();
//            return;
//        }
//
//        // 3. Phase 1: Sorting (Only runs if we haven't started firing yet)
//        if (!patternIsLocked) {
//            if (totalBalls <= 1) {
//                patternIsLocked = true; // Nothing to sort, just fire
//            } else {
//                // Define target pattern
//
//                String[] pattern = getTargetArray(targetOrder);
//                if (slot1 != null && slot2 != null) {
//                    boolean s1Match = isColorMatch(slot1, pattern[0]);
//                    boolean s2Match = isColorMatch(slot2, pattern[1]);
//
//                    if (s1Match && s2Match) {
//                        gateClose();
//                        intakeMotorIdle();
//                        patternIsLocked = true; // ORDER ACQUIRED
//                    } else {
//                        // Pattern doesn't match, keep moving
//                        performSortingCycle(slot1);
//                    }
//                } else if (slot1 != null && slot2 == null) {
//                    intake();
//                }
//                else {
//                    // We have > 1 ball total, but slot 2 is currently empty.
//                    // Keep cycling until that second ball moves up into view.
//                    performSortingCycle(slot1);
//                }
//            }
//        }
//
//        // 4. Phase 2: Firing (The "Boom Boom Boom" Phase)
//        if (patternIsLocked) {
//            if (!ballInTransfer) {
//                if (canShoot) {
//                    transfer();
//                    ballInTransfer = true;
//                } else {
//                    gateClose();
//                    intakeMotorIdle();
//                }
//            }
//        }
//
//        // 5. Shot Counter
//        if (ballInTransfer && isBeamBroken) {
//            currentShot++;
//            totalBalls--;
//            ballInTransfer = false;
//        }
//    }
//
//    /**
//     * Cycle logic: Opens gate to rotate balls,
//     * but closes gate immediately when a new ball enters Slot 1.
//     */
//    private void performSortingCycle(DetectedColor currentSlot1) {
//        boolean isSlot1Null = (currentSlot1 == null);
//
//        // Transition: Slot 1 was empty, now it's not = ball "caught"
//        if (lastSlot1WasNull && !isSlot1Null) {
//            gateClose();
//            intakeMotorIdle();
//        } else {
//            cycle(); // Opens gate and runs motor
//        }
//
//        lastSlot1WasNull = isSlot1Null;
//    }
//
//    private String[] getTargetArray(LimelightCamera.BallOrder targetOrder) {
//        if (targetOrder == LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE)
//            return new String[]{"Green", "Purple", "Purple"};
//        if (targetOrder == LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE)
//            return new String[]{"Purple", "Green", "Purple"};
//        if (targetOrder == null)
//            return new String[]{"Purple", "Purple", "Purple"};
//
//        return new String[]{"Purple", "Purple", "Green"};
//    }
//
//    private boolean isColorMatch(DetectedColor sensor, String required) {
//        if (sensor == null) return false;
//
//        if ((required.equals("Green") && sensor == DetectedColor.GREEN) || (required.equals("Purple") && sensor == DetectedColor.PURPLE)){
//            return true;
//        }
//        return false;
//    }
//


    ////////
    //SIMPLE SORTING VARIABLES
    ////////


    private String simpleCurrentPattern = null;
    private LimelightCamera.BallOrder simpleTargetPattern = null;

    private int simpleRequiredRotations = 0;
    private int simpleRotationCounter = 0;

    private boolean simpleS1HadBallLast = false;
    private boolean simpleS1LostAfterSeeing = false;

    private static final double SIMPLE_ENTRY_DEBOUNCE_MS = 600;

    private ElapsedTime simpleSortTimer = new ElapsedTime();



    public void startSimpleSort(String currentPattern,
                                LimelightCamera.BallOrder desiredPattern) {

        this.simpleCurrentPattern = currentPattern;
        this.simpleTargetPattern = desiredPattern;

        simpleSortState = SimpleSortState.CHECK_SLOTS;
    }

    public void updateSimpleSorter(DetectedColor s1,
                                    DetectedColor s2,
                                    DetectedColor s3) {

        switch (simpleSortState) {

            case IDLE:
                break;

            case CHECK_SLOTS:
                boolean s1Valid = (s1 != null);
                boolean s2Valid = (s2 != null);
                boolean s3Valid = (s3 != null);

                int valid = 0;
                if (s1Valid) valid++;
                if (s2Valid) valid++;
                if (s3Valid) valid++;


                // Only sort if slot 1 and 2 have balls
                if (valid >=2) {

                    int currentIndex = patternIndexFromString(simpleCurrentPattern);
                    int targetIndex = patternIndexFromEnum(simpleTargetPattern);

                    simpleRequiredRotations =
                            (targetIndex - currentIndex + 3) % 3;

                    simpleRotationCounter = 0;
                    simpleS1HadBallLast = true;
                    simpleS1LostAfterSeeing = false;
                    isFirstTime = true;

                    simpleSortTimer.reset();

                    if (simpleRequiredRotations == 0) {
                        simpleSortState = SimpleSortState.READY;
                    } else {
                        simpleSortState = SimpleSortState.ROTATING;
                    }

                } else {
                    simpleSortState = SimpleSortState.READY;
                }

                break;

            case ROTATING:

                // Spin sorter
                transferOn();
                setDirectionCycle();
                gateOpen();

                if (isFirstTime && simpleSortTimer.milliseconds() >400){
                    simpleSortTimer.reset();
                    isFirstTime = false;
                }

                if (simpleSortTimer.milliseconds() <= 400 && isFirstTime){
                    intake.setPower(-0.6);
                }
                else {
                    intake.setPower(1);
                }


                updateSimpleRotationCounter(s1);

                if (simpleRotationCounter >= simpleRequiredRotations) {
                    intakeMotorHalt();
                    gateClose();
                    simpleSortState = SimpleSortState.READY;
                }

                // Safety timeout
                if (simpleSortTimer.milliseconds() > 13000) {
                    intakeMotorHalt();
                    gateClose();
                    simpleSortState = SimpleSortState.READY;
                }

                break;

            case READY:

                intakeMotorHalt();
                gateClose();
                break;
        }
    }


    private void updateSimpleRotationCounter(DetectedColor s1) {

        boolean s1HasBallNow = (s1 != null);
        double now = simpleSortTimer.milliseconds();

        if (simpleS1HadBallLast && !s1HasBallNow) {
            simpleS1LostAfterSeeing = true;
            simpleSortTimer.reset();
        }

        if (simpleS1LostAfterSeeing && s1HasBallNow) {
            if (now > SIMPLE_ENTRY_DEBOUNCE_MS) {
                simpleRotationCounter++;
                simpleS1LostAfterSeeing = false;
            }
        }

        simpleS1HadBallLast = s1HasBallNow;
    }

    private int patternIndexFromString(String pattern) {

        switch (pattern) {
            case "PGP": return 0;
            case "GPP": return 1;
            case "PPG": return 2;
        }

        throw new IllegalArgumentException("Invalid pattern string: " + pattern);
    }

    private int patternIndexFromEnum(LimelightCamera.BallOrder order) {

        switch (order) {
            case PURPLE_GREEN_PURPLE: return 0;
            case GREEN_PURPLE_PURPLE: return 1;
            case PURPLE_PURPLE_GREEN: return 2;
        }

        throw new IllegalArgumentException("Invalid BallOrder enum");
    }

    // Resets the latches so we can look for the "next" set of balls
    private void resetLatches() {
        ball1Latched = false;
        ball2Latched = false;
        stabilityCounter = 0;
    }

    // Switches state and resets the timer for state-based transitions
    private void newState(IntakeState next) {
        currentState = next;
        stateTimer.reset(); // Built-in ElapsedTime method
        stabilityCounter = 0;
    }

    // Checks if the target pattern is even possible with what we have inside
    private boolean isPatternPossible(LimelightCamera.BallOrder order) {
        int reqG = 1;
        int reqP = 2;
        return (greenInventory >= reqG && purpleInventory >= reqP);
    }

    public void prepareAndStartSort() {
        newState(IntakeState.PRE_NUDGE);
    }



    public void update(boolean beamBreak, LimelightCamera.BallOrder targetOrder,
                       DetectedColor s1, DetectedColor s2, DetectedColor s3) {

        boolean anySensorSeeingBall = (s1 != null || s2 != null || s3 != null);

        switch (currentState) {

            case PRE_NUDGE:
                gateClose();
                transferOff();
                setDirectionCycle();
                intake.setPower(-0.8); //0.9

                if (stateTimer.milliseconds() > 2000) {
                    intakeMotorHalt();

                    greenInventory = 0;
                    purpleInventory = 0;
                    DetectedColor[] sensors = {s1, s2, s3};
                    for (DetectedColor color : sensors) {
                        if (color == DetectedColor.GREEN) greenInventory++;
                        else if (color == DetectedColor.PURPLE) purpleInventory++;
                    }

                    internalTotalBalls = greenInventory + purpleInventory;
                    currentShot = 0;
                    cycleCount = 0;

                    if (internalTotalBalls > 0) {
                        resetLatches();
                        newState(IntakeState.COASTING_TO_SLOT);
                    }
                }
                break;

            case IDLE:
                break;

            case COASTING_TO_SLOT:
                if (s1 != null) ball1Latched = true;
                if (s2 != null) ball2Latched = true;

                if (!ball1Latched) {
                    cycle();
                }
                else if (!ball2Latched && internalTotalBalls > 1) {
                    setDirectionCycle();
                    transferOn();
                    gateClose();
                    intake.setPower(1); //0.9
                }

                else if (s1 != null && (internalTotalBalls < 2 || s2 != null)) {
                    intakeMotorHalt();
                    newState(IntakeState.VERIFYING_ORDER);
                }
//                else {
//                    intakeMotorHalt();
//                    newState(IntakeState.VERIFYING_ORDER);
//                }
                break;

            case POSITION_NUDGE:
                gateClose();
                transferOn();
                setDirectionCycle();
                intake.setPower(1); //0.9

                if (stateTimer.milliseconds() > 4500) {
                    intakeMotorHalt();
                    nudgeCount++;
                    newState(IntakeState.VERIFYING_ORDER);
                }
                break;

            case VERIFYING_ORDER:

                // debounce nulls (allow slow motion)
                if ((s1 == null || (internalTotalBalls > 1 && s2 == null))
                        && stateTimer.milliseconds() > 5500) {

                    if (nudgeCount < 3) newState(IntakeState.POSITION_NUDGE);
                    else newState(IntakeState.FLUSHING);
                    break;
                }

                stabilityCounter++;
                if (stabilityCounter >= 3 && stateTimer.milliseconds() > 3000) {

                    boolean possible = isPatternPossible(targetOrder);
                    String[] target = getTargetArray(targetOrder);

                    boolean s1Match = isColorMatch(s1, target[0]);
                    boolean s2Match = (s2 != null) && isColorMatch(s2, target[1]);

                    if (s1Match && s2Match) {
                        newState(IntakeState.READY_TO_FIRE);
                    }
                    else if (cycleCount < 4) {
                        newState(IntakeState.FLUSHING);
                    }
                    else {
                        newState(IntakeState.READY_TO_FIRE);
                    }
                }
                break;

            case FLUSHING:
                cycle();

                // early exit if a ball arrives
                if (s1 != null && stateTimer.milliseconds() > 4500) {
                    resetLatches();
                    newState(IntakeState.COASTING_TO_SLOT);
                    break;
                }

                // longer dwell for friction systems
                if (stateTimer.milliseconds() > 6000) {

                    // only count flushes if balls exist
                    if (anySensorSeeingBall) {
                        cycleCount++;
                    }

                    if (cycleCount >= 4) {
                        newState(IntakeState.READY_TO_FIRE);
                        break;
                    }

                    resetLatches();
                    nudgeCount = 0;
                    newState(IntakeState.COASTING_TO_SLOT);
                }
                break;

            case READY_TO_FIRE:
                gateClose();
                if (canShoot) newState(IntakeState.TRANSFERRING);
                break;

            case TRANSFERRING:

                if (canShoot) {
                    transfer();
                } else {
                    intakeMotorHalt();
                }

                trackShotCount(beamBreak);

                if (currentShot >= internalTotalBalls) {
                    nudgeCount = 0;
                    cycleCount = 0;
                    currentShot = 0;
                    resetLatches();
                    newState(IntakeState.HALT);
                }
                break;

            case INTAKING:
                intake();
                break;

            case OUTTAKING:
                outtake();
                break;

            case HALT:
                intakeMotorHalt();
                newState(IntakeState.RESET);
                break;

            case RESET:
                internalTotalBalls = 0;
                greenInventory = 0;
                purpleInventory = 0;
                currentShot = 0;
                cycleCount = 0;
                nudgeCount = 0;
                stabilityCounter = 0;
                ball1Latched = false;
                ball2Latched = false;
                newState(IntakeState.IDLE);
                break;

            case CYCLE:
                cycle();
                break;

            case TEST_SHOOTING:
                transfer();
                break;
        }

        telemetry.update();
    }

//    public void initiateTransferNoSort(){
//        newState(IntakeState.TRANSFERRING);
//    }

    public IntakeState getCurrentIntakeState() {
        return currentState;
    }

    public SimpleSortState getSimpleSortState() {
        return simpleSortState;
    }





    //BEFORE CALLING: RUN INTAKE BACKWARDS TO ENSURE ALL COLOR SENSORS GET A ACCURATE READING

//    public void sort(boolean shooterBeamBrake, LimelightCamera.BallOrder targetOrder,
//                     DetectedColor colorSensor1Value,
//                     DetectedColor colorSensor2Value,
//                     DetectedColor colorSensor3Value) {
//
//        // 1. INITIALIZATION: Only happens the moment sorting starts
//        if (!isSorting) {
//            internalTotalBalls = getTotalInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value);
//
//            if (internalTotalBalls > 0) {
//                isSorting = true;
//                patternIsLocked = false;
//                currentShot = 0;
//                telemetry.addLine("--- SORT STARTED: Inventory Locked ---");
//            } else {
//                resetIndexer();
//                return; // Nothing to do
//            }
//        }
//
//        // 2. DATA SNAPSHOT (For logic/telemetry)
//        String[] pattern = getTargetArray(targetOrder);
//        boolean isBeamBroken = shooterBeamBrake;
//
//        // --- TELEMETRY ---
//        telemetry.addData("Status", patternIsLocked ? "PHASE 2: FIRING" : "PHASE 1: SORTING");
//        telemetry.addData("Inventory (Locked)", internalTotalBalls);
//        telemetry.addData("Shots Taken", currentShot);
//        telemetry.addData("Target", "%s, %s, %s", pattern[0], pattern[1], pattern[2]);
//        telemetry.addData("Real-time Sensors", "[%s] [%s] [%s]",
//                colorSensor1Value != null ? colorSensor1Value : "-",
//                colorSensor2Value != null ? colorSensor2Value : "-",
//                colorSensor3Value != null ? colorSensor3Value : "-");
//
//        // 3. COMPLETION CHECK
//        if (currentShot >= internalTotalBalls || internalTotalBalls == 0) {
//            telemetry.addLine("--- SORT COMPLETE: Resetting ---");
//            isSorting = false;
//            internalTotalBalls = 0;
//            currentShot = 0;
//            patternIsLocked = false;
//            resetIndexer();
//            return;
//        }
//
//        // 4. PHASE 1: SORTING (Ordering the balls)
//        if (!patternIsLocked) {
//            // If only 1 ball, no sorting needed
//            if (internalTotalBalls == 1) {
//                patternIsLocked = true;
//            }
//            // If sensors currently see the top two balls
//            else if (colorSensor1Value != null && colorSensor2Value != null) {
//                boolean s1Match = isColorMatch(colorSensor1Value, pattern[0]);
//                boolean s2Match = isColorMatch(colorSensor2Value, pattern[1]);
//
//                if (s1Match && s2Match) {
//                    gateClose();
//                    intakeMotorIdle();
//                    patternIsLocked = true;
//                } else {
//                    performSortingCycle(colorSensor1Value);
//                }
//            }
//            else {
//                // Balls are moving between sensors; keep cycling until they land in slots
//                performSortingCycle(colorSensor1Value);
//            }
//        }
//
//        // 5. PHASE 2: FIRING
//        if (patternIsLocked) {
//            if (!ballInTransfer) {
//                if (canShoot) {
//                    transfer();
//                    ballInTransfer = true;
//                } else {
//                    gateClose();
//                    intakeMotorIdle();
//                }
//            }
//        }
//
//        // 6. SHOT TRACKER (The only place we decrement/advance)
//        if (ballInTransfer && isBeamBroken) {
//            currentShot++;
//            ballInTransfer = false;
//            // The loop will now check if currentShot >= internalTotalBalls to finish
//        }
//    }
//
//    private void performSortingCycle(DetectedColor currentSlot1) {
//        boolean isSlot1Null = (currentSlot1 == null);
//
//        // If a ball just hit Slot 1, stop the motor to "catch" it and check color
//        if (lastSlot1WasNull && !isSlot1Null) {
//            gateClose();
//            intakeMotorIdle();
//        } else if (isSlot1Null) {
//            // Motor only runs when Slot 1 is empty to bring the next ball up
//            cycle();
//        }
//
//        lastSlot1WasNull = isSlot1Null;
//    }
//    public int getGreenInventory(DetectedColor colorSensor1Value,
//                                 DetectedColor colorSensor2Value,
//                                 DetectedColor colorSensor3Value) {
//        greenInventory = 0;
//        DetectedColor[] sensors = {colorSensor1Value, colorSensor2Value, colorSensor3Value};
//        for (DetectedColor color : sensors) {
//            if (color == DetectedColor.GREEN) greenInventory++;
//        }
//
//        return greenInventory;
//
//    }
//
//    public int getPurpleInventory(DetectedColor colorSensor1Value,
//                                 DetectedColor colorSensor2Value,
//                                 DetectedColor colorSensor3Value) {
//        purpleInventory = 0;
//        DetectedColor[] sensors = {colorSensor1Value, colorSensor2Value, colorSensor3Value};
//        for (DetectedColor color : sensors) {
//            if (color == DetectedColor.PURPLE) purpleInventory++;
//        }
//
//        return purpleInventory;
//
//    }
//
//    public int getTotalInventory(DetectedColor colorSensor1Value,
//                                 DetectedColor colorSensor2Value,
//                                 DetectedColor colorSensor3Value) {
//        return getGreenInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value) +
//                getPurpleInventory(colorSensor1Value, colorSensor2Value, colorSensor3Value);
//    }





    /**
     * Returns the target colors based on the Limelight pattern.
     */
    private String[] getTargetArray(LimelightCamera.BallOrder targetOrder) {
        if (targetOrder == LimelightCamera.BallOrder.GREEN_PURPLE_PURPLE)
            return new String[]{"Green", "Purple", "Purple"};
        if (targetOrder == LimelightCamera.BallOrder.PURPLE_GREEN_PURPLE)
            return new String[]{"Purple", "Green", "Purple"};
        if (targetOrder == null)
            return new String[]{"Purple", "Purple", "Purple"};

        return new String[]{"Purple", "Purple", "Green"};
    }

    /**
     * Checks if a specific sensor reading matches a required color string.
     */
    private boolean isColorMatch(DetectedColor sensor, String required) {
        if (sensor == null) return false;
        if (required.equals("Green") && sensor == DetectedColor.GREEN) return true;
        if (required.equals("Purple") && sensor == DetectedColor.PURPLE) return true;
        return false;
    }
    // --- Periodic update (optional) ---
//    public void update() {
////        telemetry.addData("Target RPM", targetRPM);
////        telemetry.addData("Current RPM", getCurrentRPM());
//        telemetry.addData("Servo Pos", transferDirectionSwitcher.getPosition());
//        telemetry.update();
//    }

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

    private void trackShotCount(boolean currentBeamState) {
        if (lastBeamState && !currentBeamState) {
            currentShot++;
        }
        lastBeamState = currentBeamState;
    }




    public void doSortingTelemetry(DetectedColor s1, DetectedColor s2, DetectedColor s3,LimelightCamera.BallOrder targetOrder, boolean beam) {
        telemetry.addLine("=== INTAKE SORT DIAGNOSTICS ===");
        telemetry.addData("State", "%s (%.1f ms)", currentState, stateTimer.milliseconds());
        telemetry.addData("Beam Break", beam ? "BROKEN (Ball Exiting)" : "CLEAR");


        // --- Slot 1 Logic ---
        if (s1 == DetectedColor.GREEN) {
            telemetry.addLine("Detected Color slot 1 : GREEN FN");
        } else if (s1 == DetectedColor.PURPLE) {
            telemetry.addLine("Detected Color slot 1 : Purple");
        } else {
            telemetry.addLine("Detected Color slot 1 : NONE");
        }

        // --- Slot 2 Logic ---
        if (s2 == DetectedColor.GREEN) {
            telemetry.addLine("Detected Color slot 2 : GREEN FN");
        } else if (s2 == DetectedColor.PURPLE) {
            telemetry.addLine("Detected Color slot 2 : Purple");
        } else {
            telemetry.addLine("Detected Color slot 2 : NONE");
        }

        // --- Slot 3 Logic ---
        if (s3 == DetectedColor.GREEN) {
            telemetry.addLine("Detected Color slot 3 : GREEN FN");
        } else if (s3 == DetectedColor.PURPLE) {
            telemetry.addLine("Detected Color slot 3 : Purple");
        } else {
            telemetry.addLine("Detected Color slot 3 : NONE");
        }


        telemetry.addLine("=== SIMPLE SORT DIAGNOSTICS ===");
        telemetry.addData("Simple State", simpleSortState);
        telemetry.addData("Current Pattern (String)", simpleCurrentPattern);
        telemetry.addData("Target Pattern (Enum)", simpleTargetPattern);
        telemetry.addData("Required Rotations", simpleRequiredRotations);
        telemetry.addData("Rotation Counter", simpleRotationCounter);
        telemetry.addData("S1 Had Ball Last", simpleS1HadBallLast);
        telemetry.addData("S1 Lost After Seeing", simpleS1LostAfterSeeing);
        telemetry.addData("Simple Timer (ms)", simpleSortTimer.milliseconds());


        telemetry.addLine("--- Internal State ---");
        telemetry.addData("Inventory", "T:%d | G:%d | P:%d", internalTotalBalls, greenInventory, purpleInventory);
        telemetry.addData("Shots", "%d / %d", currentShot, internalTotalBalls);
        telemetry.addData("Latches", "B1:%b B2:%b", ball1Latched, ball2Latched);
        telemetry.addData("Nudge/Cycle", "N:%d C:%d", nudgeCount, cycleCount);

        String[] target = getTargetArray(targetOrder);
        telemetry.addData("Target Order", "[%s, %s, %s]", target[0], target[1], target[2]);
    }


//    private void updateS1BallCounter(DetectedColor s1) {
//
//
//        boolean s1HasBallNow = (s1 != null);
//        double now = sussyTimer.milliseconds(); // use your ElapsedTime timer
//
//        // 1️⃣ Detect loss
//        if (s1HadBallLast && !s1HasBallNow) {
//           s1LostAfterHaving = true;
//            s1LossTime = now;
//        }
//
//        // 2️⃣ Detect new ball entry after loss and debounce time
//        if (s1LostAfterHaving  && s1HasBallNow) {
//            if (now - s1LossTime >= ENTRY_DEBOUNCE_MS) {
//                s1BallEntryCount++;
//                s1LostAfterHaving = false;
//
//            }
//        }
//
//        // 3️⃣ Update last state
//        s1HadBallLast = s1HasBallNow;
//    }

    // Sussy sorting state machine variables








}

