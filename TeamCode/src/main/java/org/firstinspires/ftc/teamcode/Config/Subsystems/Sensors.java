package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;


public class Sensors {


    // --- Core SRSHub Reference ---
   // private SRSHub hub;
    private final ElapsedTime runtime = new ElapsedTime();

    private Thread colorSensorThread;

    // --- Configured I2C Devices ---
//    private SRSHub.APDS9151 colorSensor1;
//    private SRSHub.APDS9151 colorSensor2;
//    private SRSHub.APDS9151 colorSensor3;




    // --- CALIBRATION CONSTANTS ---
//    private final int TURRET_OFFSET_DEGREES = 0;
//    private final int TURRET_MAX_TICKS = 40500;
//    private double TURRET_SKIPPED_TICKS = 0;


    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;

    NormalizedRGBA sensor1Colors;
    NormalizedRGBA sensor2Colors;
    NormalizedRGBA sensor3Colors;







    /**
     * Initialize the SRSHub and attached devices.
     */
    public void init(HardwareMap hwMap, String hubName) {

        colorSensor1 = hwMap.get(NormalizedColorSensor.class,"colorSensor1");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class,"colorSensor2");
        colorSensor3 = hwMap.get(NormalizedColorSensor.class,"colorSensor3");

//        SRSHub.Config config = new SRSHub.Config();
//
//        // Add APDS9151 to bus 1
//        config.addI2CDevice(1, new SRSHub.APDS9151());
//        config.addI2CDevice(2, new SRSHub.APDS9151());
//        config.addI2CDevice(3, new SRSHub.APDS9151());
//
//
//
//
//
//
//        // AD devices
//
////        config.setAnalogDigitalDevice(3, SRSHub.AnalogDigitalDevice.ANALOG);  // Turret Encoder
//
//
//       // config.setEncoder(5, SRSHub.Encoder.QUADRATURE); //flywheel encoder
//        //config.setEncoder(3, SRSHub.Encoder.QUADRATURE); // turret incremental
//
//        RobotLog.clearGlobalWarningMsg();
//
//        hub = hwMap.get(SRSHub.class, hubName);
//        hub.init(config);
//
//        // REQUIRED: wait until hub is fully initialized
//        waitForHubReady();

        // REQUIRED: retrieve the actual live instance from the hub
//        colorSensor1 = hub.getI2CDevice(1, SRSHub.APDS9151.class);
//        colorSensor2 = hub.getI2CDevice(2, SRSHub.APDS9151.class);
//        colorSensor3 = hub.getI2CDevice(3, SRSHub.APDS9151.class);



    }

//    /**
//     * Waits until SRSHub finishes applying configuration.
//     * Uses Thread.yield() as you wrote it.
//     */
//    public void waitForHubReady() {
//        runtime.reset();
//        while (!hub.ready() && runtime.seconds() < 2.0) {
//            Thread.yield();
//        }
//    }
//
//    public boolean isHubReady() {
//        return hub.ready();
//    }
//
//    /**
//     * Update all hub data — call once per loop.
//     */


    public void run(){
        colorSensorThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                update();
                try {
                    Thread.sleep(1);
                } catch (Exception e) {
                    break;
                }
            }
        });

        colorSensorThread.start();

    }
    private void update() {
       sensor1Colors = colorSensor1.getNormalizedColors();
       sensor2Colors = colorSensor2.getNormalizedColors();
       sensor3Colors = colorSensor3.getNormalizedColors();
    }
//
//    public boolean isHubDisconnected() {
//        return hub.disconnected();
//    }
//
//    public boolean isColorSensor1Disconnected() {
//        return colorSensor1.disconnected;
//    }
//


//    // --- APDS9151 Raw RGB ---
    public float getColor1Red() {
        return sensor1Colors.red;
    }

    public float getColor1Green() {
        return sensor1Colors.green;
    }

    public float getColor1Blue() {
        return sensor1Colors.blue;
    }
    public float getColor2Red() {
        return sensor2Colors.red;
    }

    public float getColor2Green() {
        return sensor2Colors.green;
    }

    public float getColor2Blue() {
        return sensor2Colors.blue;
    }
    public float getColor3Red() {
        return sensor3Colors.red;
    }

    public float getColor3Green() {
        return sensor3Colors.green;
    }

    public float getColor3Blue() {
        return sensor3Colors.blue;
    }
//
//    public short getColor1Proximity() {
//        return colorSensor1.proximity;
//    }
//
//    public short getColor2Proximity() {
//        return colorSensor2.proximity;
//    }
//    public short getColor3Proximity() {
//        return colorSensor3.proximity;
//    }
//
//
//    // --- Digital Beam Breaks (AD pins 1 & 2) ---
//
//
//
//
//
//
//    // --- Raw Analog Encoder Values ---
////    public double getAnalogEncoder1Value() {
////        return hub.readAnalogDigitalDevice(3);
////
////    }
//
////    public double getRawTurretTicks(){
////        return hub.readEncoder(3).position * -1;
////
////    }
//
////    public void rezeroTurretEncoder() {
////        // Read the current absolute hardware position
////        //basically this sets the offset to what the turret thinks its current position is
////        TURRET_SKIPPED_TICKS = getRawTurretTicks();
////    }
////
////    public double getSketchTurretPosition(){
////        return Range.scale(
////                getRawTurretTicks() - TURRET_SKIPPED_TICKS,
////                0, TURRET_MAX_TICKS,              // Input Range
////                0, 360  // Output Range (CORRECTED)
////        ) ;
////    }
//
////    public double getAnalogEncoder2Value() {
////        return hub.readAnalogDigitalDevice(4);
////    }
//
//    // --- Calibrated Positions (clamping logic unchanged) ---
////    public int getTurretPosition() {
////        double normalizedValue = getAnalogEncoder1Value();
////        int pos = (int) (Math.round(normalizedValue/3.2 * 360) + TURRET_OFFSET_DEGREES) % 360;
////
//////        if (pos < 0) {pos += 360;}
//////        if (pos >  360) {pos -= 360;}
////
////        return pos;
////    }
//
////    public int getHoodPosition() {
////        double normalizedValue = getAnalogEncoder2Value();
////        int pos = (int) (Math.round(normalizedValue/3.2 * 360) + HOOD_OFFSET_DEGREES) % 360;
////
////        if (pos < 0) {pos += 360;}
////        if (pos >  360) {pos -= 360;}
////
////        return pos;
////    }
//
//    // flywheel velo in ticks/sec
////    public double getFlywheelVelo() {
////        return hub.readEncoder(5).velocity;
////    }
//
//

    public void stop(){
        if (colorSensorThread != null) {
            colorSensorThread.interrupt();
        }
    }


    public DetectedColor getDetectedColor(float red, float blue, float green) {
        float r = red;
        float g = green;
        float b = blue;
        float greenDifferenceOffset = 95;

        // --- NOTHING DETECTED ---
//        if (r < 120 && g < 120 && b < 120) {
//            return null;
//        }

        // --- GREEN: green strongest ---
        if (g - greenDifferenceOffset > r && g - greenDifferenceOffset > b) {
            return DetectedColor.GREEN;
        }

        // --- PURPLE: both G and B > R ---
        else if ( g + b > 350 && (g + b / 2) > r) {
            return DetectedColor.PURPLE;
        }

        else if (g - 40 > r && g - 40 > b){
            return DetectedColor.GREEN;
        }
        else if (b + 12 >= g && g - 25 > r ){
            return DetectedColor.PURPLE;
        }

        // --- everything else counts as NONE ---
        return null;
    }
}