package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tejas Narayanan on 11/19/16.
 */

@TeleOp(name= "Sweeper Servo Test", group = "TeleOp")
public class SweeperTest extends OpMode {
    Servo sweeper; // Sweeper motor

    @Override
    public void init() {
        sweeper = hardwareMap.servo.get("sweeper_m");
    }

    @Override
    public void loop() {
        if(gamepad1.y){
            sweeper.setPosition(1); //surgical tubing forward, counterclockwise
        } else if(gamepad1.a){
            sweeper.setPosition(0); //surgical tubing backward
        } else if(gamepad1.x){
            sweeper.setPosition(5); //stop surgical tubing
        } else if(gamepad1.b){
            sweeper.setPosition(0.51); //stop surgical tubing
        }
    }
}
