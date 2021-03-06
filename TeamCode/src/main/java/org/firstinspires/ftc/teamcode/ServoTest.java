package org.firstinspires.ftc.teamcode;

/**
 * Created by ShruthiJaganathan on 10/5/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name="Servo Test", group="Test Opmode")
public class ServoTest extends OpMode{

    Servo servo;
    boolean activateServo = true;
    double servoPos;

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

        if (gamepad1.y) {
            if(gamepad1.dpad_up){
                servo.setPosition(0.00001);
            } else if(gamepad1.dpad_down){
                servo.setPosition(1);
            } else {
                servo.setPosition(0.5);
            }
        }
        else {
            servo.setPosition(0.5);
        }

        telemetry.addData("Servo Position", servo.getPosition());
    }

    public void stop(){

    }
}
