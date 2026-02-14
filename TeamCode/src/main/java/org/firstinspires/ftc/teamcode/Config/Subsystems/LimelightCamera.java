package org.firstinspires.ftc.teamcode.Config.Subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.util.List;

@Configurable
public class LimelightCamera {

    public Limelight3A limelightCamera;


    public static double HEADING_KP_TX = 0.019; //0.02
    public static double HEADING_KI_TX = 0.0;
    public static double HEADING_KD_TX = 0.0001;
    public static double HEADING_KF_TX = 0.0;
    public static double ROTATION_MIN_POWER = 0.0;

    private PIDFController pidf;



    /*
     RECOMENDED NEW VALUES - YALL SHOULD TEST THO

     public static double HEADING_KP_TX = 0.045 ; // Increased for faster response
    public static double HEADING_KI_TX = 0.0;     // Keep at 0 for now
    public static double HEADING_KD_TX = 0.003;  // Added for damping/braking
    public static double ROTATION_MIN_POWER = 0.07; // Added to overcome motor friction
    */


    /**
     * PIPELINE NUMBER GUIDE -
     * 3 is for the april tags on the goals
     * 5 is for the obilisk
     * 6 is for detector pipeline
     */

    public double finalRotation = 0;
    public double error = 0;


    public LimelightCamera(HardwareMap hardwareMap) {
        limelightCamera = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelightCamera == null) {
            throw new IllegalStateException("❌ Limelight not found in hardwareMap! Check config name 'limelight'.");
        }
        limelightCamera.start();
        pidf = new PIDFController(HEADING_KP_TX, HEADING_KI_TX, HEADING_KD_TX, HEADING_KF_TX);
        // Set the target to 0 (we want the Limelight Tx error to be zero)
        pidf.setSetPoint(0.0);
    }

    /**
     * Returns the latest LLResult safely
     */
    public LLResult getResult() {
        LLResult result = limelightCamera.getLatestResult();
        return result;
    }

    /**
     * Updates the error based on the latest tag result
     */
    public double updateError() {
        LLResult result = getResult();
        if (result != null && result.isValid()) {
            error = -1* (result.getTyNC() );//+3.5
        } else {
            error = 0;
        }
        return error;
    }

    /**
     * Auto-align PID rotation value based on limelight data
     */
    public double autoAlign() {
        limelightCamera.pipelineSwitch(3);

        LLResult result = getResult();
        pidf.setPIDF(HEADING_KP_TX, HEADING_KI_TX, HEADING_KD_TX, HEADING_KF_TX);

        if (result != null && result.isValid()) {

            error = updateError();
            finalRotation = pidf.calculate(error);

            if (Math.abs(finalRotation) > 0 && Math.abs(finalRotation) < ROTATION_MIN_POWER) {
                finalRotation = Math.signum(finalRotation) * ROTATION_MIN_POWER;
            }


            finalRotation = Range.clip(finalRotation, -1.0, 1.0);

        } else {
            finalRotation = 0.0;
            pidf.reset();
        }

        return finalRotation;
    }

    public double calculateDistanceToGoal(double robotX, double robotY, double goalX, double goalY) {
        // Change in X and Y
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Apply the distance formula: d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
        return Math.hypot(deltaX, deltaY);
    }


    /**
     * Enum for ball orders
     */
    public enum BallOrder {
        GREEN_PURPLE_PURPLE, // id 21
        PURPLE_GREEN_PURPLE, // id 22
        PURPLE_PURPLE_GREEN  // id 23
    }

    BallOrder currentOrder = null;

    /**
     * Detects ball order based on AprilTag IDs 21-23; switches pipeline to detection pipeline
     */
//    public BallOrder detectBallOrder() {
//        limelightCamera.pipelineSwitch(5); // detection pipeline for ALL april tags
//
//        LLResult result = getResult();
//        BallOrder detectedOrder = null; // default
//
////        if (result != null && result.isValid()) {
//        if (result != null ) {
//            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
//                int id = fr.getFiducialId();
//
//                if (id == 21) {
//                    detectedOrder = BallOrder.GREEN_PURPLE_PURPLE;
//                }
//                if (id == 22) {
//                    detectedOrder = BallOrder.PURPLE_GREEN_PURPLE;
//                }
//                if (id == 23) {
//                    detectedOrder = BallOrder.PURPLE_PURPLE_GREEN;
//                }
//
//            }
//            currentOrder = detectedOrder;
//            return detectedOrder;
//        }
//        return null;
//    }

    public BallOrder detectBallOrder() {
        limelightCamera.pipelineSwitch(5);

        LLResult result = getResult();

        if (result == null || !result.isValid()) {
            return currentOrder;
        }

        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            switch (fr.getFiducialId()) {
                case 21:
                    return currentOrder = BallOrder.GREEN_PURPLE_PURPLE;
                case 22:
                    return currentOrder = BallOrder.PURPLE_GREEN_PURPLE;
                case 23:
                    return currentOrder = BallOrder.PURPLE_PURPLE_GREEN;
            }
        }

        return currentOrder; // keep last known order
    }

    public BallOrder getCurrentBallOrder(){
        return currentOrder;
    }

    public double[] getLimeLightTargetDegrees() {
        limelightCamera.pipelineSwitch(3);
        double[] targetDegrees = new double[2];

        if (getResult() != null) {
            for (LLResultTypes.FiducialResult fiducial : getResult().getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((id == 20) || (id == 24)) {

                    targetDegrees[0] = fiducial.getTargetXDegrees();
                    targetDegrees[1] = fiducial.getTargetYDegrees();

                    return targetDegrees;
                }
            }
        }

        return null;
    }

    public double[] getArtifactsOnRamp() {
        limelightCamera.pipelineSwitch(5); //idk we need a detector pipeline
        double[] totalResults = new double[3];

        List<LLResultTypes.ClassifierResult> classifierResults = getResult().getClassifierResults();
        for (LLResultTypes.ClassifierResult cr : classifierResults) {

            double totalDetectedArtifacts = classifierResults.size();
            totalResults[0] = totalDetectedArtifacts;

            String className = cr.getClassName();
            int purpleCount = 0;
            int greenCount = 0;

            // Use exact string comparison (case-sensitive) for your model's labels
            if (className.equals("purple") && cr.getConfidence() > 0.6) {
                purpleCount++;
            } else if (className.equals("green") && cr.getConfidence() > 0.6) {
                greenCount++;
            }

            totalResults[1] = greenCount;
            totalResults[2] = purpleCount;
        }

        return totalResults;
    }


}