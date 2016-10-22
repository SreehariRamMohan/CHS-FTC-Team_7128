package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ShruthiJaganathan on 10/19/16.
 */
public class TeleOp_v1  extends OpMode{
    Servo servo1 = hardwareMap.servo.get("servo_1"); // Beacon Press servo
    Servo servo2 = hardwareMap.servo.get("servo_2"); // Ball Shooter servo

    DcMotor motor1 = hardwareMap.dcMotor.get("motor_1"); // Ball Shooter motor
    DcMotor rightMotor = hardwareMap.dcMotor.get("right_m"); // Right side wheels
    DcMotor leftMotor = hardwareMap.dcMotor.get("left_m"); // Left Side Wheels
    DcMotor sweeper = hardwareMap.dcMotor.get("sweeper_m"); // Sweeper motor

    @Override
    public void init() {

    }

    @Override
    public void loop() {
    }
}
