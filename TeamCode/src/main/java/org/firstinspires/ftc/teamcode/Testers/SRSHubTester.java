package org.firstinspires.ftc.teamcode.Testers;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Config.Util.SRSHub;

@Disabled
@TeleOp(name = "HubTest", group = "TEST")
public class SRSHubTester extends LinearOpMode {




    public void runOpMode() throws InterruptedException {
//        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(
//                telemetry,
//                FtcDashboard.getInstance().getTelemetry()
//        );

        // All ports default to NONE, buses default to empty
        SRSHub.Config config = new SRSHub.Config();

//        config.setEncoder(
//                1,
//                SRSHub.Encoder.PWM
//        );
//
//        config.setEncoder(
//                2,
//                SRSHub.Encoder.QUADRATURE
//        );


        config.addI2CDevice(1,new SRSHub.APDS9151());
        config.setAnalogDigitalDevice(4, SRSHub.AnalogDigitalDevice.ANALOG);  //ad3/4   pin a

//        config.addI2CDevice(
//                1,
//                new SRSHub.GoBildaPinpoint(
//                        -50,
//                        -75,
//                        19.89f,
//                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
//                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD
//                )
//        );

        RobotLog.clearGlobalWarningMsg();

        SRSHub hub = hardwareMap.get(
                SRSHub.class,
                "SRSHub"
        );

        hub.init(config);

        while (!hub.ready());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            hub.update();

            if (hub.disconnected()) {
                telemetry.addLine("srshub disconnected");
            } else {

                double normalizedValue = hub.readAnalogDigitalDevice(4);
                int pos = (int) (Math.round(normalizedValue * 360)) % 360;



                SRSHub.APDS9151 colorSensor = hub.getI2CDevice(1, SRSHub.APDS9151.class);
                telemetry.addData("Color Sensor Blue Reading:",colorSensor.blue);
                telemetry.addData("Color Sensor Red Reading:",colorSensor.red);
                telemetry.addData("Color Sensor Green Reading:",colorSensor.green);
                telemetry.addData("Analog Melonbotics Encoder Reading RAW:", hub.readAnalogDigitalDevice(4));
                telemetry.addData("Analog Melonbotics Encoder Reading:", pos);
//                telemetry.addData(
//                        "encoder 1 position",
//                        hub.readEncoder(1).position
//                );
//
//                telemetry.addData(
//                        "encoder 2 position",
//                        hub.readEncoder(2).position
//                );
//
//              telemetry.addData(
//                        "encoder 2 velocity",
//                        hub.readEncoder(2).velocity
//                );

//                SRSHub.GoBildaPinpoint pinpoint = hub.getI2CDevice(
//                        1,
//                        SRSHub.GoBildaPinpoint.class
//                );
//
//                if (!pinpoint.disconnected) {
//                    multipleTelemetry.addData(
//                            "pose x (mm)",
//                            pinpoint.xPosition
//                    );
//
//                    multipleTelemetry.addData(
//                            "pose y (mm)",
//                            pinpoint.yPosition
//                    );
//
//                    multipleTelemetry.addData(
//                            "pose heading (rad)",
//                            pinpoint.hOrientation
//                    );
//                }


            }

            telemetry.update();
        }
    }
}
