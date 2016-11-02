package org.firstinspires.ftc.teamcode;

/**
 * Created by ShruthiJaganathan on 10/5/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Test Opmode")
public class ServoTest extends OpMode{

    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    /*
     > 0.5 is counterclockwise, 0.5 is full stop, < 0.5 is clockwise
     Minimum is 0, Maximum is 1
     */
    public void loop() {

        if(gamepad1.dpad_right){
            servo.setPosition(0.2);
        } else if(gamepad1.dpad_left){
            servo.setPosition(0.7);
        } else {
            servo.setPosition(0.5);
        }

        telemetry.addData("Servo Position", servo.getPosition());
    }

    public void stop(){

    }
}
