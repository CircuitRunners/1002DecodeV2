package org.firstinspires.ftc.teamcode.Config.Util;

public class ValidShootingZoneChecker {
    // Helper class for a coordinate pair
        private static class Point {
            public double x;
            public double y;
            public Point(double x, double y) {
                this.x = x;
                this.y = y;
            }
        }

        // 1. Define the Fixed Vertices for Triangle 1 (The Goal-Sids Shooting Zone)
        // Vertices: (0, 144), (72, 72), and (144, 144)
        private final Point V1_A = new Point(0.0, 144.0);
        private final Point V1_B = new Point(72.0, 72.0);
        private final Point V1_C = new Point(144.0, 144.0);

        // 2. Define the Fixed Vertices for Triangle 2 (The Far/Human-Player side Zone)
        // Vertices: (48, 0), (72, 24), and (96, 0)
        private final Point V2_A = new Point(48.0, 0.0);
        private final Point V2_B = new Point(72.0, 24.0);
        private final Point V2_C = new Point(96.0, 0.0);

        // ------------------------------------------------------------------------
        // Cross-Product (Signed Area)
        // ------------------------------------------------------------------------

    private double crossProduct(Point P1, Point P2, Point P3) {
        // This calculates the 2D cross-product (or signed area).
        // The sign (+, -, or 0) determines which side of the line P1->P2 the point P3 lies on.
        return (P2.x - P1.x) * (P3.y - P1.y) - (P2.y - P1.y) * (P3.x - P1.x);
    }


    /**
     * Checks if a point is inside a single triangle defined by its vertices.
     */
    private boolean isInsideSingleTriangle(Point P_robot, Point V_A, Point V_B, Point V_C) {
        // EXPLANATION:
        // A point is inside a triangle if and only if it lies on the *same side*
        // of all three lines formed by the triangle's edges.

        // Compute the cross-products for the three sub-triangles.
        double sign1 = crossProduct(V_A, V_B, P_robot);
        double sign2 = crossProduct(V_B, V_C, P_robot);
        double sign3 = crossProduct(V_C, V_A, P_robot);

        // Check if all signs are the same (all non-negative OR all non-positive).
        // Zero (0) is included for points exactly on the boundary line.
        boolean allNonNegative = (sign1 >= 0) && (sign2 >= 0) && (sign3 >= 0);
        boolean allNonPositive = (sign1 <= 0) && (sign2 <= 0) && (sign3 <= 0);

        return allNonNegative || allNonPositive;
    }


        /**
         * Checks if the robot is inside Triangle 1 OR Triangle 2.
         *
         * @param currentRobotX The X-coordinate from your odometry/pose (e.g., pose.x)
         * @param currentRobotY The Y-coordinate from your odometry/pose (e.g., pose.y)
         * @return true if the robot is inside EITHER triangle.
         */
        public boolean isInsideShootingZone(double currentRobotX, double currentRobotY) {
            Point robotPose = new Point(currentRobotX, currentRobotY);

            // Check against Triangle 1
            boolean inTriangle1 = isInsideSingleTriangle(robotPose, V1_A, V1_B, V1_C);

            // Check against Triangle 2
            boolean inTriangle2 = isInsideSingleTriangle(robotPose, V2_A, V2_B, V2_C);

            // Return true if the pose is in EITHER triangle (OR logic)
            return inTriangle1 || inTriangle2;
        }
    }

