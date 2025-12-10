package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Config.Util.DetectedColor;
import org.firstinspires.ftc.teamcode.Config.Util.SRSHub;


public class Sensors {


    // --- Core SRSHub Reference ---
    private SRSHub hub;
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Configured I2C Devices ---
    private SRSHub.APDS9151 colorSensor1;
    private SRSHub.APDS9151 colorSensor2;
    private SRSHub.APDS9151 colorSensor3;

    // --- CALIBRATION CONSTANTS ---
    private final int TURRET_OFFSET_DEGREES = 0;
    private final int TURRET_CLAMP_MAX = 360;
    private final int HOOD_OFFSET_DEGREES = 0;
    private final int HOOD_CLAMP_MAX = 360;



    /**
     * Initialize the SRSHub and attached devices.
     */
    public void init(HardwareMap hwMap, String hubName) {
        SRSHub.Config config = new SRSHub.Config();

        // Add APDS9151 to bus 1
        config.addI2CDevice(1, new SRSHub.APDS9151());
        config.addI2CDevice(2, new SRSHub.APDS9151());
        config.addI2CDevice(3, new SRSHub.APDS9151());

        // AD devices
        config.setAnalogDigitalDevice(1, SRSHub.AnalogDigitalDevice.DIGITAL); // Beam Break 1

        config.setAnalogDigitalDevice(3, SRSHub.AnalogDigitalDevice.ANALOG);  // Turret Encoder
        config.setAnalogDigitalDevice(4, SRSHub.AnalogDigitalDevice.ANALOG);  // Hood Encoder

        config.setEncoder(1, SRSHub.Encoder.QUADRATURE); //flywheel encoder

        RobotLog.clearGlobalWarningMsg();

        hub = hwMap.get(SRSHub.class, hubName);
        hub.init(config);

        // REQUIRED: wait until hub is fully initialized
        waitForHubReady();

        // REQUIRED: retrieve the actual live instance from the hub
        colorSensor1 = hub.getI2CDevice(1, SRSHub.APDS9151.class);
        colorSensor2 = hub.getI2CDevice(2, SRSHub.APDS9151.class);
        colorSensor3 = hub.getI2CDevice(3, SRSHub.APDS9151.class);

    }

    /**
     * Waits until SRSHub finishes applying configuration.
     * Uses Thread.yield() as you wrote it.
     */
    public void waitForHubReady() {
        runtime.reset();
        while (!hub.ready() && runtime.seconds() < 2.0) {
            Thread.yield();
        }
    }

    public boolean isHubReady() {
        return hub.ready();
    }

    /**
     * Update all hub data — call once per loop.
     */
    public void update() {
        if (!isHubDisconnected()) {
            hub.update();
        }
    }

    public boolean isHubDisconnected() {
        return hub.disconnected();
    }

    public boolean isColorSensor1Disconnected() {
        return colorSensor1.disconnected;
    }

    // --- APDS9151 Raw RGB ---
    public int getColor1Red() {
        return colorSensor1.red;
    }

    public int getColor1Green() {
        return colorSensor1.green;
    }

    public int getColor1Blue() {
        return colorSensor1.blue;
    }
    public int getColor2Red() {
        return colorSensor2.red;
    }

    public int getColor2Green() {
        return colorSensor2.green;
    }

    public int getColor2Blue() {
        return colorSensor2.blue;
    }
    public int getColor3Red() {
        return colorSensor3.red;
    }

    public int getColor3Green() {
        return colorSensor3.green;
    }

    public int getColor3Blue() {
        return colorSensor3.blue;
    }

    public short getColor1Proximity() {
        return colorSensor1.proximity;
    }

    public short getColor2Proximity() {
        return colorSensor2.proximity;
    }
    public short getColor3Proximity() {
        return colorSensor3.proximity;
    }


    // --- Digital Beam Breaks (AD pins 1 & 2) ---
    public boolean isBeamBroken1() {
        return hub.readAnalogDigitalDevice(1) > 0.5;
    }



    // --- Raw Analog Encoder Values ---
    public double getAnalogEncoder1Value() {
        return hub.readAnalogDigitalDevice(3);
    }

    public double getAnalogEncoder2Value() {
        return hub.readAnalogDigitalDevice(4);
    }

    // --- Calibrated Positions (clamping logic unchanged) ---
    public int getTurretPosition() {
        double normalizedValue = getAnalogEncoder1Value();
        int pos = (int) (Math.round(normalizedValue/3.2 * 360) + TURRET_OFFSET_DEGREES) % 360;

        if (pos < 0) {pos += 360;}
        if (pos >  360) {pos -= 360;}

        return pos;
    }

    public int getHoodPosition() {
        double normalizedValue = getAnalogEncoder2Value();
        int pos = (int) (Math.round(normalizedValue/3.2 * 360) + HOOD_OFFSET_DEGREES) % 360;

        if (pos < 0) {pos += 360;}
        if (pos >  360) {pos -= 360;}

        return pos;
    }

    // flywheel velo in ticks/sec
    public double getFlywheelVelo(){
        return hub.readEncoder(1).velocity;
    }


    public DetectedColor getDetectedColor(int red, int blue, int green) {
        int r = red;
        int g = green;
        int b = blue;

        // --- NOTHING DETECTED ---
        if (r < 120 && g < 120 && b < 120) {
            return null;
        }

        // --- GREEN: green strongest ---
        if (g > r && g > b) {
            return DetectedColor.GREEN;
        }

        // --- PURPLE: both G and B > R ---
        if (g > r && b > r) {
            return DetectedColor.PURPLE;
        }

        // --- everything else counts as NONE ---
        return null;
    }
}