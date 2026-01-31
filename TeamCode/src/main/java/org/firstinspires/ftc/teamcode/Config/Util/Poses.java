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

    // SHOOT_POSITION_GOAL_SIDE
//    public static final AlliancePose shootPositionGoalSide = new AlliancePose(
//            new Pose(86.5, 80.0, Math.toRadians(36.5)),     // Red: X=144-59.5=84.5, Theta=180-135=45
//            new Pose(57.5, 80, Math.toRadians(136))         // Blue
//    );

    // SHOOT_POSITION_GOAL_SIDE_2
    public static final AlliancePose shootPositionGoalSide2 = new AlliancePose(
            new Pose(100.5, 104.5, Math.toRadians(42)),     // Red: X=144-48=96, Theta=180-132.5=47.5
            new Pose(43.5, 104.5, Math.toRadians(138))       // Blue
    );

    public static final AlliancePose shootPositionGoalSide3 = new AlliancePose(
            new Pose(105.5, 104.5, Math.toRadians(42)),     // Red: X=144-48=96, Theta=180-132.5=47.5
            new Pose(38.5, 104.5, Math.toRadians(138))       // Blue
    );

//    public static final AlliancePose shootPositionGoalSide2LasthHot = new AlliancePose(
//            new Pose(100.5, 119.5, Math.toRadians(22)),     // Red: X=144-48=96, Theta=180-132.5=47.5
//            new Pose(43.5, 119.5, Math.toRadians(158))      // Blue
//    );


    // CONTROL_POINT_LINE_1_FOR_SHOOT_POSE_2
    public static final AlliancePose controlPointLine1ForShootPose2 = new AlliancePose(
            new Pose(91.0, 80, Math.toRadians(0)),
            new Pose(53, 80, Math.toRadians(180))
    );

    // PICKUP_LINE_1
    public static final AlliancePose pickupLine1 = new AlliancePose(
            new Pose(124.5, 81, Math.toRadians(0)),
            new Pose(15.5, 81, Math.toRadians(180)) //maybe change x to x = 20.5 if still no work idk
    );


    // PICKUP_LINE_2
    public static final AlliancePose pickupLine2 = new AlliancePose(
            new Pose(140, 56, Math.toRadians(0)),
            new Pose(5, 56, Math.toRadians(180))
    );

    // LINE_2_CONTROL_POINT
    public static final AlliancePose line2ControlPoint = new AlliancePose(
            new Pose(82.0, 53.0, Math.toRadians(0)),
            new Pose(64, 54, Math.toRadians(180))
    );



    // PICKUP_LINE_3
    public static final AlliancePose pickupLine3 = new AlliancePose(
            new Pose(140, 33, Math.toRadians(0)),
            new Pose(5, 33, Math.toRadians(180))
    );

    // LINE_3_CONTROL_POINT
    public static final AlliancePose line3ControlPoint = new AlliancePose(
            new Pose(81.0, 26.5, Math.toRadians(0)),
            new Pose(64, 28, Math.toRadians(180))
    );

    // LINEUP_AT_GATE
    public static final AlliancePose lineupAtGate = new AlliancePose(
            new Pose(119.0, 61.0, Math.toRadians(0)),
            new Pose(24, 68, Math.toRadians(180))
    );

    public static final AlliancePose openGate = new AlliancePose(
            new Pose(141.5, 85, Math.toRadians(180)),
            new Pose(3.5, 85, Math.toRadians(0))
    );
    public static final AlliancePose openGateAutoIntake = new AlliancePose(
            new Pose(130.0, 62.5, Math.toRadians(42)),
            new Pose(10, 62.5, Math.toRadians(138))
    );
    public static final AlliancePose openGateAutoIntakeControlPoint = new AlliancePose(
            new Pose(47, 37, Math.toRadians(42)),
            new Pose(93, 37, Math.toRadians(138))
    );



    // LINE_4_CONTROL_POINT


    // PICKUP_LINE_1_TO_GATE_CONTROL_POINT
    public static final AlliancePose pickupLine1ToGateControlPoint = new AlliancePose(
            new Pose(103.0, 77.5, Math.toRadians(180)),
            new Pose(39, 77.5, Math.toRadians(0))
    );




    // TELEOP
    public static final AlliancePose teleopDefaultPose = new AlliancePose(
            new Pose(72, 72, Math.toRadians(270)),
            new Pose(72, 72, Math.toRadians(90))
    );

    // FAR SIDE
    public static final AlliancePose startPoseFarSide = new AlliancePose(
            new Pose(60, 8, Math.toRadians(180)),
            new Pose(84, 8, Math.toRadians(0))
    );
    public static final AlliancePose shootPositionFarSide = new AlliancePose(
            new Pose(60, 24, Math.toRadians(120)),
            new Pose(84, 24, Math.toRadians(60))
    );
    public static final AlliancePose humanPlayerIntake = new AlliancePose(
            new Pose(7, 8, Math.toRadians(180)),
            new Pose(137, 8, Math.toRadians(0))
    );
    public static final AlliancePose backUpPoint = new AlliancePose(
            new Pose(15, 8, Math.toRadians(180)),
            new Pose(129, 8, Math.toRadians(0))
    );
    public static final AlliancePose humanPlayerControlPoint = new AlliancePose(
            new Pose(96.3, 7.2),
            new Pose(47.7, 7.2)
    );
    public static final AlliancePose intake3ControlPoint = new AlliancePose(
            new Pose(61.1, 38.5),
            new Pose(82.9, 38.5)
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