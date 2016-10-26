package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by robotics on 10/26/16.
 */

public class Color_Sensor_Test extends LinearOpMode {

    boolean activateServo = true;
    double servoPosition;
    Servo servo1 = hardwareMap.servo.get("servo_1");
    ColorSensor cr = hardwareMap.colorSensor.get("mr");
    public static final double MAX_POSITION = 1;
    public static final double MIN_POSITION = 0;
    double currentTime;
    LED led;
    TouchSensor t;
    /*
    servo1 = hardwareMap.servo.get("servo_1");
    cr = hardwareMap.colorSensor.get("mr");
    */
   public void runOpMode() {

       led = hardwareMap.led.get("led");
       t = hardwareMap.touchSensor.get("t");

       // wait for start();

       float hsvValues[] =  {0, 0, 0};
       final float values[] = hsvValues;
       final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

       enabledLed(t.isPressed());

       Color.RGBToHSV(cr.red()*8, cr.green()*8, cr.blue() * 8, hsvValues);

       double redV = cr.red();
       double blueV = cr.blue();

       telemetry.addData("Clear", cr.alpha());
       telemetry.addData("Red  ", cr.red());
       telemetry.addData("Green", cr.green());
       telemetry.addData("Blue ", cr.blue());
       telemetry.addData("Hue", hsvValues[0]);
       double time = this.time;
       while (this.time - time < 30) {
           if(blueV-redV >= 50) {
               telemetry.addData("This is blue!" , cr.blue());
           } else if (redV - blueV >= 50){
               telemetry.addData("This is red" , cr.red());
           }
       }

   }

    private void enabledLed(boolean value) {cr.enableLed(value);}

}




