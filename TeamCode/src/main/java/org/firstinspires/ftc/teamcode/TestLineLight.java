package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Tejas Narayanan on 2/2/17.
 */

@Autonomous(name= "Test Line Light", group = "Autonomous")
public class TestLineLight extends LinearOpMode {

    OpticalDistanceSensor ds;

    public void runOpMode() throws InterruptedException {

        ds = hardwareMap.opticalDistanceSensor.get("ds");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("ODS Light", ds.getRawLightDetected());
            telemetry.update();
        }
    }
}
