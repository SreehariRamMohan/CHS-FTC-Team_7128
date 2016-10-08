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

public class BeaconPressTest extends OpMode{

    boolean activateServo = true;
    double servoPosition;
    Servo servo1;
    ColorSensor cr;
    public static final double MAX_POSITION = 1;
    public static final double MIN_POSITION = 0;
    double currentTime;
    LED led;
    TouchSensor t;

    public void init(){
        servo1 = hardwareMap.servo.get("servo_1");
        cr = hardwareMap.colorSensor.get("mr");
    }

    public void loop() {

        if (activateServo) {
            servoPosition = servo1.getPosition();
            servo1.setPosition(servoPosition);
        }

        led = hardwareMap.led.get("led");
        t = hardwareMap.touchSensor.get("t");

        //waitForStart();

        float hsvValues[] = {0, 0, 0};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        enableLed(t.isPressed());

        Color.RGBToHSV(cr.red() * 8, cr.green() * 8, cr.blue() * 8, hsvValues);

        double redV = cr.red();
        double blueV = cr.blue();

        telemetry.addData("Clear", cr.alpha());
        telemetry.addData("Red  ", cr.red());
        telemetry.addData("Green", cr.green());
        telemetry.addData("Blue ", cr.blue());
        telemetry.addData("Hue", hsvValues[0]);

        if (blueV - redV >= 50) {
            //if blue is our team color
            //press the button , move closer
            telemetry.addData("This is blue!", cr.blue());
        } else {
            currentTime = this.time;
            while (this.time - currentTime < 1) {
                servo1.setPosition(0);
            }
            //press the button, move closer
            telemetry.addData("This is red!", cr.red());
        }
    }

    private void enableLed(boolean value) {
        cr.enableLed(value);
    }

}


