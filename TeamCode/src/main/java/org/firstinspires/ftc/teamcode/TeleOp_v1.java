package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ShruthiJaganathan on 10/19/16.
 */


public class TeleOp_v1  extends OpMode{
    Servo servo1; // Beacon Press servo
    Servo servo2; // Ball Shooter servo
    Servo sweeper; // Sweeper motor

    DcMotor motor1; // Ball Shooter motor
    DcMotor rightMotor; // Right side wheels
    DcMotor leftMotor; // Left Side Wheels

    @Override
    public void init() {
        servo1 = hardwareMap.servo.get("servo_1");
        servo2 = hardwareMap.servo.get("servo_2");
        motor1 = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("right_m");
        leftMotor = hardwareMap.dcMotor.get("left_m");
        sweeper = hardwareMap.servo.get("sweeper_m");
    }

    @Override
    public void loop() {
        // Gamepad 1

        if(gamepad1.right_stick_y == -1) {
          rightMotor.setPower(-1); //moves right wheels forward
        } else if(gamepad1.right_stick_y == 1) {
            rightMotor.setPower(1); //moves right wheels backward
        } else {
            rightMotor.setPower(0); //stops right wheels
        }

        if(gamepad1.left_stick_y == -1){
            leftMotor.setPower(1); //moves left wheels forward
        } else if(gamepad2.left_stick_y == 1){
            leftMotor.setPower(-1); //moves left wheels backward
        } else {
            leftMotor.setPower(0); //stops left wheels
        }

        if(gamepad1.y){
            sweeper.setPosition(1); //surgical tubing forward
        } else if(gamepad1.a){
            sweeper.setPosition(-1); //surgical tubing backward
        } else if(gamepad1.x){
            sweeper.setPosition(0); //stop surgical tubing
        }

        //gamepad 2

        if(gamepad2.dpad_left){
            servo1.setPosition(0.7); //beacon press left
        } else if(gamepad2.dpad_right){
            servo1.setPosition(0.3); //beacon press right
        } else {
            servo1.setPosition(0.5); //caution: stop
        }

        if(gamepad2.y){
            servo2.setPosition(0.7); //ball shooter servo up
        } else if(gamepad2.a){
            servo2.setPosition(0.3); //ball shooter servo  down
        } else if(gamepad2.x){
            servo2.setPosition(0.5); //stop ball shooter servo
        }

        if(gamepad2.right_stick_y == 1){
            motor1.setPower(1); //turn on ball shooting motor
        } else if(gamepad2.right_stick_y == -1){
            motor1.setPower(-1); //turn motor backwards
        } else if(gamepad2.b){
            motor1.setPower(0); //stop ball shooting motor
        }


    }
}
