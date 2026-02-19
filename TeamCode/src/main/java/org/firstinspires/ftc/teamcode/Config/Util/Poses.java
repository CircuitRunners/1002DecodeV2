package org.firstinspires.ftc.teamcode.Config.Util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Poses {

    public enum Alliance { RED, BLUE }

    public static class AlliancePose {
        private final Pose redPose;
        private final Pose bluePose;

        // Explicit values for Red and Blue
        public AlliancePose(Pose redPose, Pose bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        // Mirror convenience: Red is computed automatically
        public static AlliancePose mirror(Pose bluePose) {
            return new AlliancePose(bluePose.mirror(), bluePose);
        }

        public Pose get(Alliance alliance) {
            return (alliance == Alliance.RED) ? redPose : bluePose;
        }
    }

    // =======================
    // Alliance selection
    // =======================
    private static Alliance currentAlliance = Alliance.RED;
    private static Gamepad previousGamepad = new Gamepad();

    public static Alliance getAlliance() {
        return currentAlliance;
    }
    private static void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    public static Pose get(AlliancePose p) {
        return p.get(currentAlliance);
    }

    public static void updateAlliance(Gamepad g, Telemetry telemetry) {
        if (g.dpad_up && !previousGamepad.dpad_up) setAlliance(Alliance.RED);
        else if (g.dpad_down && !previousGamepad.dpad_down) setAlliance(Alliance.BLUE);
        previousGamepad.copy(g);

//        telemetry.addLine("--- Alliance Selector ---");
//        telemetry.addData("Current Alliance", currentAlliance);
//        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
//        telemetry.update();
    }


    // =======================
    // Example poses
    // =======================

    // ️Mirrored point (Red auto-computed)


    //  Non-mirrored point (explicit different coords)
  /*EXAMPLE*/  private static final AlliancePose preloadPose = new AlliancePose(
            new Pose(42, 65.83, 0),     // Red
            new Pose(-40, 60, Math.toRadians(10))  // Blue custom
    );


    /**
     * Common Terms:
     * Line 1 - Closest Line To Goal
     * Line 3- Closest line to human player side
     * Line 4 - Actually in human player zone
     */

    public static final AlliancePose startPoseGoalSide = new AlliancePose(
            new Pose(112.0, 135.5, Math.toRadians(0)),      // Red: X=144-32=112, Theta=180-180=0
            new Pose(32, 135.5, Math.toRadians(180))        // Blue
    );

    public static final AlliancePose getMotif = new AlliancePose(
            new Pose(90, 102.5, Math.toRadians(120)),      // Red: X=144-32=112, Theta=180-180=0
            new Pose(54, 102.5, Math.toRadians(60))        // Blue
    );
    // SHOOT_POSITION_GOAL_SIDE
//    public static final AlliancePose shootPositionGoalSide = new AlliancePose(
//            new Pose(86.5, 80.0, Math.toRadians(36.5)),     // Red: X=144-59.5=84.5, Theta=180-135=45
//            new Pose(57.5, 80, Math.toRadians(136))         // Blue
//    );

    // SHOOT_POSITION_GOAL_SIDE_2
//    public static final AlliancePose shootPositionGoalSide2 = new AlliancePose(
//            new Pose(100.5, 107, Math.toRadians(37)),     // Red: X=144-48=96, Theta=180-132.5=47.5
//            new Pose(43.5, 107, Math.toRadians(145))       // Blue
//    );
//
//    public static final AlliancePose shootPositionGoalSide3 = new AlliancePose(
//            new Pose(105.5, 104.5, Math.toRadians(42)),     // Red: X=144-48=96, Theta=180-132.5=47.5
//            new Pose(38.5, 104.5, Math.toRadians(138))       // Blue
//    );

    public static final AlliancePose shootPositionGoalSide2 = new AlliancePose(
            new Pose(95.5, 89, Math.toRadians(46)),
            new Pose(58.5, 89, Math.toRadians(136.5)) // 144 - 100.5 = 43.5
    );

    public static final AlliancePose shootPositionGoalSide15Ball = new AlliancePose(
            new Pose(92.5, 89, Math.toRadians(49.5)),
            new Pose(51.5, 89, Math.toRadians(130.5)) // 144 - 100.5 = 43.5
    );

    public static final AlliancePose shootPositionGoalSide15BallLastTime = new AlliancePose(
            new Pose(95.5, 101, Math.toRadians(46)),
            new Pose(58.5, 101, Math.toRadians(136.5)) // 144 - 100.5 = 43.5
    );

    public static final AlliancePose shootPositionGoalSide3 = new AlliancePose(
            new Pose(105.5, 104.5, Math.toRadians(42)),
            new Pose(38.5, 104.5, Math.toRadians(138)) // 144 - 105.5 = 38.5
    );



    // CONTROL_POINT_LINE_1_FOR_SHOOT_POSE_2
//    public static final AlliancePose controlPointLine1ForShootPose2 = new AlliancePose(
//            new Pose(96, 67, Math.toRadians(0)),
//            new Pose(48, 67, Math.toRadians(180))
//    );
    public static final AlliancePose controlPointLine1ForShootPose2 = new AlliancePose(
            new Pose(68.4, 81, Math.toRadians(0)),
            new Pose(75.6, 81, Math.toRadians(180))
    );
    // PICKUP_LINE_1
    public static final AlliancePose pickupLine1 = new AlliancePose(
            new Pose(138.5, 84, Math.toRadians(0)),
            new Pose(10, 84, Math.toRadians(180)) //maybe change x to x = 20.5 if still no work idk
    );
    public static final AlliancePose chainPickupLine1 = new AlliancePose(
            new Pose(134, 84, Math.toRadians(0)),
            new Pose(12, 84, Math.toRadians(180)) //maybe change x to x = 20.5 if still no work idk
    );

    public static final AlliancePose pickupLineOne15Ball = new AlliancePose(
            new Pose(131, 82, Math.toRadians(0)),
            new Pose(17, 84, Math.toRadians(180)) //maybe change x to x = 20.5 if still no work idk
    );
    public static final AlliancePose chainPickupLineOne15Ball = new AlliancePose(
            new Pose(127, 84, Math.toRadians(0)),
            new Pose(22, 84, Math.toRadians(180)) //maybe change x to x = 20.5 if still no work idk
    );


    // PICKUP_LINE_2
    public static final AlliancePose pickupLine2 = new AlliancePose(
            new Pose(148, 58, Math.toRadians(0)),
            new Pose(3.5, 59, Math.toRadians(180))
    );
    public static final AlliancePose chainPickupLine2 = new AlliancePose(
            new Pose(140.5, 58, Math.toRadians(0)),
            new Pose(13, 59, Math.toRadians(180))
    );

    // LINE_2_CONTROL_POINT
    public static final AlliancePose line2ControlPoint = new AlliancePose(
            new Pose(84, 55.5, Math.toRadians(0)),
            new Pose(60, 57, Math.toRadians(180))
    );



    // PICKUP_LINE_3
    public static final AlliancePose pickupLine3 = new AlliancePose(
            new Pose(148, 34, Math.toRadians(0)),
            new Pose(5, 36, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine3Far = new AlliancePose(
            new Pose(139, 32, Math.toRadians(0)),
            new Pose(5, 34, Math.toRadians(180))
    );

    public static final AlliancePose chainPickupLine3 = new AlliancePose(
            new Pose(139, 35, Math.toRadians(0)),
            new Pose(13, 36, Math.toRadians(180))
    );

    // LINE_3_CONTROL_POINT
    public static final AlliancePose line3ControlPoint = new AlliancePose(
            new Pose(64.5, 27.5, Math.toRadians(0)),
            new Pose(74.4, 30.5, Math.toRadians(180))
    );



//    // LINEUP_AT_GATE
//    public static final AlliancePose lineupAtGate = new AlliancePose(
//            new Pose(119.0, 61.0, Math.toRadians(0)),
//            new Pose(24, 68, Math.toRadians(180))
//    );
//
//    public static final AlliancePose openGate = new AlliancePose(
//            new Pose(143, 83, Math.toRadians(180)),
//            new Pose(1, 83, Math.toRadians(0))
//    );
//    public static final AlliancePose openGateAutoIntakeJames = new AlliancePose(
//            new Pose(130.0, 62.5, Math.toRadians(42)),
//            new Pose(10, 62.5, Math.toRadians(138))
//    );
//    public static final AlliancePose openGateAutoIntakeControlPointJames = new AlliancePose(
//            new Pose(47, 37, Math.toRadians(42)),
//            new Pose(93, 37, Math.toRadians(138))
//    );
//
//    public static final AlliancePose openGateHighCycle = new AlliancePose(
//            new Pose(133,60,Math.toRadians(45)),
//            new Pose(11,60, Math.toRadians(135))
//    );
//
//    public static final AlliancePose openGateHighCycleControlPoint = new AlliancePose(
//            new Pose(87.5, 74, Math.toRadians(42)),
//            new Pose(56.5, 74, Math.toRadians(138))
//    );
//    public static final AlliancePose intakeFromGateHighCycle = new AlliancePose(
//            new Pose(135,50,Math.toRadians(90)),
//            new Pose(9,50, Math.toRadians(90))
//    );

    public static final AlliancePose openGate = new AlliancePose(
            new Pose(131, 84.5, Math.toRadians(180)),
            new Pose(21, 72.5, Math.toRadians(0)) // 144 - 143 = 1 ✅
    );
    public static final AlliancePose openGate12Ball = new AlliancePose(
            new Pose(131.2, 69.2, Math.toRadians(0)),
            new Pose(12.8, 69.2, Math.toRadians(180)) // 144 - 143 = 1 ✅
    );
    public static final AlliancePose openGate12BallControlPoint = new AlliancePose(
            new Pose(91.3, 71.1, Math.toRadians(180)),
            new Pose(52.7, 71.1, Math.toRadians(0)) // 144 - 143 = 1 ✅
    );



    public static final AlliancePose openGateHighCycle = new AlliancePose(
            new Pose(142, 62, Math.toRadians(45)),
            new Pose(19.5, 58.5, Math.toRadians(135)) // correct
    );
    public static final AlliancePose openGateRamTech = new AlliancePose(
            new Pose(119, 52.7, Math.toRadians(70)),
            new Pose(25, 52.7, Math.toRadians(110))
    );

    public static final AlliancePose openGateHighCycleControlPoint = new AlliancePose(
            new Pose(84,57, Math.toRadians(42)), // new Pose(103.5, 74.5, Math.toRadians(42)),
            new Pose(60.5, 74, Math.toRadians(138)) // correct
    );

    public static final AlliancePose intakeFromGateHighCycle = new AlliancePose(
            new Pose(149.5, 56.5, Math.toRadians(65)),
            new Pose(22, 49, Math.toRadians(115)) // correct
    );






    // TELEOP
    public static final AlliancePose teleopDefaultPose = new AlliancePose(
            new Pose(72, 72, Math.toRadians(270)),
            new Pose(72, 72, Math.toRadians(90))
    );

    // FAR SIDE
    public static final AlliancePose startPoseFarSide = new AlliancePose(
            new Pose(84, 9, Math.toRadians(0)),
            new Pose(60, 9, Math.toRadians(180))
    );
    public static final AlliancePose startPose15Ball = new AlliancePose(
            new Pose(136.5, 121.5, Math.toRadians(38.51)), //118.5
            new Pose(25.5, 118.5, Math.toRadians(141.5))
    );
    public static final AlliancePose shootPositionFarSide = new AlliancePose(
            new Pose(86, 11.5, Math.toRadians(0)),
            new Pose(58, 11.5, Math.toRadians(180))
    );
    public static final AlliancePose humanPlayerIntake = new AlliancePose(
            new Pose(134, 12, Math.toRadians(0)),
            new Pose(9, 12, Math.toRadians(180))
    );
    public static final AlliancePose humanPlayerIntakeRam = new AlliancePose(
            new Pose(134, 15, Math.toRadians(0)),
            new Pose(9, 15, Math.toRadians(180))
    );
    public static final AlliancePose backUpPoint = new AlliancePose(
            new Pose(124, 12, Math.toRadians(0)),
            new Pose(15, 12, Math.toRadians(180))
    );
//    public static final AlliancePose humanPlayerControlPoint = new AlliancePose(
//            new Pose(96.3, 7.2),
//            new Pose(47.7, 7.2)
//    );
    public static final AlliancePose intake3ControlPoint = new AlliancePose(
            new Pose(61.1, 40),
            new Pose(82.9, 40)
    );


    public static final AlliancePose intake3ControlPointFar = new AlliancePose(
            new Pose(81.1, 40),
            new Pose(62.9, 40)
    );


    //15 ball shenanegans
    public static final AlliancePose fifteenBallOpenGateControlPoint = new AlliancePose(
            new Pose(113.7, 62),
            new Pose(30.3, 62)
    );

    public static final AlliancePose fifteenBallLine4ControlPoint1 = new AlliancePose(
            new Pose(142.6, 49),
            new Pose(1.4, 49)
    );

    public static final AlliancePose fifteenBallLine4ControlPoint2 = new AlliancePose(
            new Pose(133.4, 56),
            new Pose(10.6, 56)
    );

    public static final AlliancePose fifteenBallPickupLine4 = new AlliancePose(
            new Pose(135.5, 8.5,Math.toRadians(270)),
            new Pose(8.5, 8.5,Math.toRadians(270))
    );

    //all of the sussy dillan stuff
    public static final AlliancePose susInitialShoot = new AlliancePose(
            new Pose(128.5, 89, Math.toRadians(49.5)),
            new Pose(56.5, 86, Math.toRadians(130.5)) // 144 - 100.5 = 43.5
    );
    public static final AlliancePose susPickupLine2 = new AlliancePose(
            new Pose(140.5, 58, Math.toRadians(0)),
            new Pose(3.5, 60, Math.toRadians(180))
    );
    public static final AlliancePose susLine2ControlPoint = new AlliancePose(
            new Pose(84, 57, Math.toRadians(0)),
            new Pose(45, 58, Math.toRadians(180))
    );
    public static final AlliancePose susOpenGateHighCycleControlPoint = new AlliancePose(
            new Pose(83.5, 74, Math.toRadians(42)),
            new Pose(40, 55, Math.toRadians(138)) // correct
    );
    public static final AlliancePose susPreLine1 = new AlliancePose(
            new Pose(128.5, 89, Math.toRadians(49.5)),
            new Pose(51.5, 84, Math.toRadians(130.5)) // 144 - 100.5 = 43.5
    );
    public static final AlliancePose susPostLine1 = new AlliancePose(
            new Pose(128.5, 89, Math.toRadians(49.5)),
            new Pose(55, 85, Math.toRadians(130.5)) // 144 - 100.5 = 43.5
    );
    public static final AlliancePose susLine3ControlPoint = new AlliancePose(
            new Pose(69.6, 30.5, Math.toRadians(0)),
            new Pose(64.7, 27.7, Math.toRadians(180))
    );
    public static final AlliancePose susFinal = new AlliancePose(
            new Pose(69.6, 30.5, Math.toRadians(0)),
            new Pose(55, 105, Math.toRadians(180))
    );


    // =======================
    // Cross-OpMode pose storage
    // =======================

    /**
     * This holds the "last known" robot pose across OpModes.
     * Autonomous should set this at the end, TeleOp should read it on init.
     */
    private static Pose lastPose = null;

    /** Save a pose (e.g. at end of Auto) */
    public static void savePose(Pose pose) {
        lastPose = pose;
    }

    /** Get the correct starting pose: last saved if available, else default startPose */
    public static Pose getStartingPose() {
        return (lastPose != null) ? lastPose : get(teleopDefaultPose);
    }

    /** Clear saved pose (optional, e.g. between practice runs) */
    public static void reset() {
        lastPose = null;
    }
    /* usage:

    at end of autonomous:

    Poses.savePose(follower.getPose());

    at start of teleop:
    follower.setStartingPose(Poses.getStartingPose());
     */
}