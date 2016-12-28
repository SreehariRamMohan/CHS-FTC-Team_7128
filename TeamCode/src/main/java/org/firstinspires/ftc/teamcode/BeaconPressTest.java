package org.firstinspires.ftc.teamcode;

/**
 * Created by ShruthiJaganathan on 10/1/16.
 */

// Test comment
//Test comment 2 after pull

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name= "Beacon Press Test", group = "Autonomous")
public class BeaconPressTest extends LinearOpMode {

    Servo servo1;
    ColorSensor cr;
    /*
    servo1 = hardwareMap.servo.get("servo_1");
    cr = hardwareMap.colorSensor.get("mr");
    */
    @Override
    public void runOpMode()  throws InterruptedException, NullPointerException {

        servo1 = hardwareMap.servo.get("beacon");
        cr = hardwareMap.colorSensor.get("mr");

        servo1.setPosition(0.5);

        waitForStart();

        //final float values[] = hsvValues;
        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        cr.enableLed(false);

        //Color.RGBToHSV(cr.red() * 8, cr.green() * 8, cr.blue() * 8, hsvValues);
        while(opModeIsActive()) {
            double redV = cr.red();
            double blueV = cr.blue();

            telemetry.addData("Red  ", cr.red());
            telemetry.addData("Green", cr.green());
            telemetry.addData("Blue ", cr.blue());


            if (blueV - redV > 0.5) {
                //if blue is our team color
                //press the button , move closer
                servo1.setPosition(0.1);
                telemetry.addData("This is blue!", cr.blue());
            } else if (redV - blueV > 0.5) {
                servo1.setPosition(0.9);
                //press the button, move closer
                telemetry.addData("This is red!", cr.red());
            }
            else {
                servo1.setPosition(0.5);
            }
            /* // sree prototype possible fix for spazzing out
            if(cr.blue() > 0 && cr.red() == 0) {
                servo1.setPosition(0.2);
                //drive forward to push the button;
                telemetry.addData("BLUE", cr.blue());
                servo1.setPosition(0.5);
                telemetry.addData("Blue", cr.blue());
            } else if(cr.red() > 0 && cr.blue() == 0) {
                servo1.setPosition(0.7);
                // drive forward and push the beacon;
                telemetry.addData("RED", cr.red());
                servo1.setPosition(0.5);
                telemetry.addData("Red", cr.red());
            } */

            telemetry.update();

            idle();
        }

    }


}


