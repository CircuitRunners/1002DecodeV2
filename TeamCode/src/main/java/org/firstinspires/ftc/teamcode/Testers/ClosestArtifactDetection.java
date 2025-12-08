package org.firstinspires.ftc.teamcode.Testers;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

// 1. REMOVE @Disabled TAG AND UPDATE NAME
@TeleOp(name = "Artifact Detection Tester", group = "TEST")
public class ClosestArtifactDetection extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // --- GREEN LOCATOR ---
        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true) // Set to true to see the detected boxes on the preview
                .setBlurSize(3)
                .build();

        // --- PURPLE LOCATOR ---
        // **NOTE: These values must be tuned for your specific environment and ball color!**
        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true) // Set to true to see the detected boxes on the preview
                .setBlurSize(3)
                .build();

        // --- VISION PORTAL ---
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(greenLocator, purpleLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Wait for the OpMode to be initialized or started
        while (opModeIsActive() || opModeInInit())
        {
            // 2. COMBINE BLOBS FROM BOTH PROCESSORS
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();

            List<ColorBlobLocatorProcessor.Blob> allBlobs = new ArrayList<>();
            allBlobs.addAll(greenBlobs);
            allBlobs.addAll(purpleBlobs);


            // 3. FILTER BLOBS
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    500, 20000, allBlobs);  // Filter out very small (noise) or very large (entire frame) blobs.


            // 4. SORT BY CONTOUR AREA DESCENDING: This is how you snap to the CLOSEST one
            // The closest object will have the largest pixel area.
            // This prevents oscillation between two equally large, distant targets.
            ColorBlobLocatorProcessor.Util.sortByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    SortOrder.DESCENDING, allBlobs);


            // --- TELEMETRY OUTPUT ---
            telemetry.addData("Status", opModeIsActive() ? "Running" : "Initialized");
            telemetry.addData("Total Blobs Found (After Filter)", allBlobs.size());
            telemetry.addLine("---------------------------------------------");
            telemetry.addLine("Area  Density Aspect Arc Circle Center (Closest Targets)");

            int count = 0;
            for(ColorBlobLocatorProcessor.Blob b : allBlobs)
            {
                if (count >= 5) break; // Limit the output to the top 5 closest targets

                RotatedRect boxFit = b.getBoxFit();

                // Determine which processor found this blob (a quick check is all we can do)
                String colorTag = greenBlobs.contains(b) ? "GREEN" : (purpleBlobs.contains(b) ? "PURPLE" : "UNKNOWN");

                telemetry.addLine(String.format("%5d  %4.2f  %5.2f %3d %5.3f (%3d,%3d) - %s",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) b.getArcLength(), b.getCircularity(), (int) boxFit.center.x, (int) boxFit.center.y, colorTag));
                count++;
            }


            final double CAMERA_CENTER_X = 160.0;
            final double K_P = 0.006; // Starting Kp value: 0.006 is a good, conservative start
            final double TURN_POWER_LIMIT = 0.3; // Limit the max turn speed

            if (!allBlobs.isEmpty()) {
                // Display the center of the CLOSEST target (the first one in the sorted list)
                RotatedRect closestBox = allBlobs.get(0).getBoxFit();
                String closestColor = greenBlobs.contains(allBlobs.get(0)) ? "GREEN" : (purpleBlobs.contains(allBlobs.get(0)) ? "PURPLE" : "UNKNOWN");

                telemetry.addLine("---------------------------------------------");
                telemetry.addData("CLOSEST TARGET:", "%s at X=%d, Y=%d", closestColor, (int) closestBox.center.x, (int) closestBox.center.y);
            } else {
                telemetry.addLine("No targets found in the size range.");
            }

            if (!allBlobs.isEmpty()) {
                RotatedRect closestBox = allBlobs.get(0).getBoxFit();
                double targetX = closestBox.center.x; // X-coordinate of the closest target

                // 1. Calculate the Error
                // Positive Error -> Target is left of center -> Need to turn LEFT (positive rotation)
                // Negative Error -> Target is right of center -> Need to turn RIGHT (negative rotation)
                double error = CAMERA_CENTER_X - targetX;

                // 2. Calculate Proportional Turn Power
                double turnPower = error * K_P;

                // 3. Limit the Turn Power for smooth movement
                if (Math.abs(turnPower) > TURN_POWER_LIMIT) {
                    turnPower = Math.signum(turnPower) * TURN_POWER_LIMIT;
                }

                String closestColor = greenBlobs.contains(allBlobs.get(0)) ? "GREEN" : (purpleBlobs.contains(allBlobs.get(0)) ? "PURPLE" : "UNKNOWN");
                telemetry.addLine("---------------------------------------------");
                telemetry.addData("CLOSEST TARGET:", "%s at X=%.1f", closestColor, targetX);
                telemetry.addData("ERROR:", "%.1f", error);
                telemetry.addData("TURN POWER:", "%.3f", turnPower);
            }

            telemetry.update();
            sleep(50);
        }
    }
}
