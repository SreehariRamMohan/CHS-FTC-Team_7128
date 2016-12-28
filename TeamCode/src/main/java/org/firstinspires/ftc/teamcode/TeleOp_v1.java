package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ShruthiJaganathan on 10/19/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="TeleOp v1", group="Test Opmode")
public class TeleOp_v1  extends OpMode{
    Servo beaconServo; // Beacon Press servo
    Servo flipperServo; // Ball Shooter servo

    DcMotor sweeper; // Sweeper motor
    DcMotor ballLeftMotor; // Ball Shooter left motor
    DcMotor ballRightMotor; // Ball Shooter right motor
    DcMotor rightMotor; // Right side wheels
    DcMotor leftMotor; // Left Side Wheels
    double currTime; // timer

    @Override
    public void init() {
        beaconServo = hardwareMap.servo.get("beacon");
        //flipperServo = hardwareMap.servo.get("flipper");
        //ballRightMotor = hardwareMap.dcMotor.get("ball_right");
        //ballLeftMotor = hardwareMap.dcMotor.get("ball_left");
        rightMotor = hardwareMap.dcMotor.get("right_m");
        leftMotor = hardwareMap.dcMotor.get("left_m");
        sweeper = hardwareMap.dcMotor.get("sweeper_m");
        currTime = this.time;
    }

    @Override
    public void loop() {
        // Gamepad 1
        currTime = this.time;

        if (gamepad1.right_stick_y == -1) {
            rightMotor.setPower(-1);
        }
        else if (gamepad1.right_stick_y == 1) {
            rightMotor.setPower(0.5);
        }
        else {
            rightMotor.setPower(0);
        }

        if (gamepad1.left_stick_y == -1) {
            leftMotor.setPower(1);
        }
        else if (gamepad1.left_stick_y == 1) {
            leftMotor.setPower(-0.5);
        }
        else {
            leftMotor.setPower(0);
        }


        /*
        if(gamepad1.y){
            while(this.time - currTime < 10) {
                sweeper.setPosition(1); //surgical tubing forward
            }
        } else if(gamepad1.a){
            while(this.time - currTime < 10) {
                sweeper.setPosition(-1); //surgical tubing backward
            }
        }

*/

        if(gamepad2.dpad_down){
            sweeper.setPower(1); //surgical tubing forward
        } else if(gamepad2 .dpad_up) {
            sweeper.setPower(-1); // surgical tubing backward
        } else {
            sweeper.setPower(0); //else: stop surgical tubing
        }


        if(gamepad2.dpad_left){
            beaconServo.setPosition(0); //beacon press left
        } else if(gamepad2.dpad_right){
            beaconServo.setPosition(1); //beacon press right
        } else {
            beaconServo.setPosition(0.5); //caution: stop
        }

        //gamepad 2

        /*
        if(gamepad2.dpad_left){
            beaconServo.setPosition(0.7); //beacon press left
        } else if(gamepad2.dpad_right){
            beaconServo.setPosition(0.3); //beacon press right
        } else {
            beaconServo.setPosition(0.5); //caution: stop
        }
        */


        /*

        if(gamepad2.y){
            flipperServo.setPosition(0.7); //ball shooter servo up
        } else if(gamepad2.a){
            flipperServo.setPosition(0.3); //ball shooter servo  down
        } else if(gamepad2.x){
            flipperServo.setPosition(0.5); //stop ball shooter servo
        }

        if(gamepad2.right_trigger > 0.5){
            ballLeftMotor.setPower(1); //turn on ball shooting motor
            ballRightMotor.setPower(-1);
        } else if(gamepad2.b){
            ballLeftMotor.setPower(0); //stop ball shooting motor
            ballRightMotor.setPower(0);
        }

        */

        /*else if(gamepad2.right_stick_y == -1){
            ballLeftMotor.setPower(-1); //turn motor backwards
            ballRightMotor.setPower(1);
        }
        */



    }
}
