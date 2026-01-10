package org.firstinspires.ftc.teamcode.Testers;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.teamcode.Config.Subsystems.Sensors;

import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;





@TeleOp(group = "TEST")

public class SRSTeleop extends OpMode {





    private Sensors sensors = new Sensors();



    private boolean initializationFailed = false;



// The name of the SRSHub in your robot configuration file

    private static final String HUB_NAME = "SRSHub";



    /**

     * Called once when the driver presses INIT.

     * Used for all hardware configuration and initialization.

     */

    @Override

    public void init() {

        telemetry.addData("Status", "Initializing Sensors...");

        telemetry.update();





        try {

// The Sensors class handles finding and configuring the SRSHub itself

            sensors.init(hardwareMap, HUB_NAME);

            telemetry.addData("Status", "Initialization Successful!");

        } catch (Exception e) {

            telemetry.addData("Status", "FATAL ERROR during initialization!");

            telemetry.addData("Error", e.getMessage());

            initializationFailed = true;

        }

        telemetry.update();

    }



    /**

     * Called repeatedly after INIT but before START is pressed.

     */

    @Override

    public void init_loop() {

        if (initializationFailed) {

            telemetry.addData("WARNING", "Fix the error and restart the OpMode.");

            return;

        }





// Continuously update status during the INIT phase to see hub readiness

        if (sensors.isHubReady()) {

            sensors.update();

        }



// Using the new methods from your Sensors class

        telemetry.addData("Hub Status", sensors.isHubDisconnected() ? "DISCONNECTED (Error)" :

                (sensors.isHubReady() ? "Ready (Awaiting Start)" : "Waiting for Config..."));

        telemetry.addData("Instructions", "Press START to begin reading data.");

        telemetry.update();

    }



    /**

     * Called once when the driver presses START.

     */

    @Override

    public void start() {

// --- CRITICAL CHECK: Throw error if starting before hub is ready ---

        if (!sensors.isHubReady()) {

// Throwing a RuntimeException here will immediately halt the OpMode

            throw new RuntimeException("SRSHub is not ready. Please wait for 'Ready (Awaiting Start)' status before pressing START.");

        }



// Clean up initialization data on the screen

        telemetry.clearAll();

    }



    /**

     * Called repeatedly after START is pressed until STOP is pressed.

     * This is the main control loop for reading and displaying data.

     */

    @Override

    public void loop() {

        if (initializationFailed) {

            return; // Don't try to read sensors if they failed to init

        }



// 1. Call update() to fetch the latest bulk data from the SRSHub

        sensors.update();



// 2. Check for hub connection status (Should be ready if we passed the start check)

        if (sensors.isHubDisconnected()) {

            telemetry.addData("CRITICAL", "SRSHub Disconnected! Check cable.");

            telemetry.update();

            return; // Skip reading sensor data if disconnected

        }





// 3. Read Calibrated Analog Encoder Data

        telemetry.addData("--- ANALOG ENCODERS (DEGREES) ---", "---");

       // telemetry.addData("1. Turret Position", sensors.getTurretPosition());
        telemetry.addData("1. Turret RAW Position", sensors.getRawTurretTicks());
        telemetry.addData("1. Turret SKETCH Position", sensors.getSketchTurretPosition());

        telemetry.addData("2. Flywheel Velo", sensors.getFlywheelVelo());



// 4. Read Digital Beam Break Data

//        telemetry.addData("--- DIGITAL INPUTS (BEAM BREAKS) ---", "---");
//
//        telemetry.addData("3. Beam Break 1 (AD-1)", sensors.isBeamBroken() ? "BROKEN (HIGH)" : "BLOCKED (LOW)");
//




// 5. Read and interpret Color Sensor Data

// Reference the enum from within the Sensors class

        DetectedColor color = sensors.getDetectedColor(sensors.getColor1Red(), sensors.getColor1Blue(),sensors.getColor1Green());
        DetectedColor color2 = sensors.getDetectedColor(sensors.getColor2Red(), sensors.getColor2Blue(),sensors.getColor2Green());
        DetectedColor color3 = sensors.getDetectedColor(sensors.getColor3Red(), sensors.getColor3Blue(),sensors.getColor3Green());

    if (color == DetectedColor.GREEN){
        telemetry.addLine("Detected Color slot 1 : GREEN FN");
    }
    else  if (color == DetectedColor.PURPLE){
        telemetry.addLine("Detected Color slot 1 : Purple");
    }

    if (color2 == DetectedColor.GREEN){
        telemetry.addLine("Detected Color slot 2 : GREEN FN");
    }
    else  if (color2 == DetectedColor.PURPLE){
        telemetry.addLine("Detected Color slot 2 : Purple");
    }
    if (color3 == DetectedColor.GREEN){
            telemetry.addLine("Detected Color slot 3 : GREEN FN");
        }
    else  if (color3 == DetectedColor.PURPLE){
        telemetry.addLine("Detected Color slot 3 : Purple");
    }





        telemetry.addData("--- COLOR SENSOR (I2C) ---", "---");

       // telemetry.addData("5. Detected Color", color != null ? color.toString() : "NONE");

        telemetry.addData("Raw R/G/B", "%d / %d / %d",

                sensors.getColor1Red(),

                sensors.getColor1Green(),

                sensors.getColor1Blue());

        telemetry.addData("Proximity", sensors.getColor1Proximity());

        telemetry.addData("Raw R/G/B", "%d / %d / %d",

                sensors.getColor2Red(),

                sensors.getColor2Green(),

                sensors.getColor2Blue());

        telemetry.addData("Proximity", sensors.getColor2Proximity());

        telemetry.addData("Raw R/G/B", "%d / %d / %d",

                sensors.getColor3Red(),

                sensors.getColor3Green(),

                sensors.getColor3Blue());

        telemetry.addData("Proximity", sensors.getColor3Proximity());







// 6. Update telemetry screen

        telemetry.update();

    }



    /**

     * Called once when the driver presses STOP.

     */

    @Override

    public void stop() {

        telemetry.addData("Status", "OpMode Stopped.");

        telemetry.update();

    }

}