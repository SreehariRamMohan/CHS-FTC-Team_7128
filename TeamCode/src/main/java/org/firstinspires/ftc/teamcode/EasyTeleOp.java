package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tejas Narayanan on 3/1/17.
 */
@TeleOp(name="International TeleOp", group="Test Opmode")
public class EasyTeleOp extends OpMode {
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
        flipperServo = hardwareMap.servo.get("flipper");
        ballRightMotor = hardwareMap.dcMotor.get("ball_right");
        ballLeftMotor = hardwareMap.dcMotor.get("ball_left");
        rightMotor = hardwareMap.dcMotor.get("right_m");
        leftMotor = hardwareMap.dcMotor.get("left_m");
        sweeper = hardwareMap.dcMotor.get("sweeper_m");
        currTime = this.time;
    }

    public void loop() {
        // Gamepad 1
        currTime = this.time;

        //start test

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if(gamepad1.dpad_up) {
            rightMotor.setPower(1);
            leftMotor.setPower(1);
        } else if(gamepad1.dpad_down) {
            rightMotor.setPower(-0.5);
            leftMotor.setPower(-0.5);
        } else if(gamepad1.dpad_left) {
            leftMotor.setPower(-1);
            rightMotor.setPower(1);
        } else if(gamepad1.dpad_right) {
            leftMotor.setPower(1);
            rightMotor.setPower(-1);
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        //intake code
        if(gamepad1.right_stick_y == 1) {
            sweeper.setPower(1);
        } else if(gamepad1.right_stick_y == -1) {
            sweeper.setPower(-1);
        } else {
            sweeper.setPower(0);
        }

        if(gamepad1.left_stick_y == -1) {
            ballLeftMotor.setPower(1); //turn on ball shooting motor
            ballRightMotor.setPower(-1);
        } else if(gamepad1.left_stick_y == 1){
            ballLeftMotor.setPower(-1); //turn on ball shooting motor
            ballRightMotor.setPower(1);
        } else {
            ballLeftMotor.setPower(0); //turn on ball shooting motor
            ballRightMotor.setPower(0);
        }

        if(gamepad1.a) {
            flipperServo.setPosition(0.5);
        } else {
            flipperServo.setPosition(1);
        }

        if(gamepad1.x) {
            beaconServo.setPosition(0);
        } else if(gamepad1.b) {
            beaconServo.setPosition(1);
        } else {
            beaconServo.setPosition(0.5);
        }





    }

}
