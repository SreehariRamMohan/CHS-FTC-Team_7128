package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tejas Narayanan on 11/19/16.
 */

@TeleOp(name= "Sweeper Servo Test", group = "TeleOp")
public class SweeperTest extends OpMode {
    DcMotor sweeper; // Sweeper motor

    @Override
    public void init() {
        sweeper = hardwareMap.dcMotor.get("sweeper_m");
    }

    @Override
    public void loop() {
        if(gamepad1.y){
            sweeper.setPower(1); //surgical tubing forward, counterclockwise
        } else if(gamepad1.a){
            sweeper.setPower(-1); //surgical tubing backward
        } else if(gamepad1.x){
            sweeper.setPower(0); //stop surgical tubing
        }
    }
}
