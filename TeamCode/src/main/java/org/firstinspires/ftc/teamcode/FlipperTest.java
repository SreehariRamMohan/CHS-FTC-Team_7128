package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ShruthiJaganathan on 12/29/16.
 */
@TeleOp(name="Flipper Test", group="Test Opmode")
public class FlipperTest extends OpMode{
    Servo flipper;

    @Override
    public void init() {
        flipper = hardwareMap.servo.get("flipper");
    }

    @Override
    public void loop(){
     if(gamepad1.a){
         flipper.setPosition(0.5);
     } else if(gamepad1.b){
         flipper.setPosition(0);
     } else{
         flipper.setPosition(1);
     }
    }
}
