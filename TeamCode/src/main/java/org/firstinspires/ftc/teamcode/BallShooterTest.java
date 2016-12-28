package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ShruthiJaganathan on 10/16/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Ball Shooter Test TeleOp", group="Test Opmode")
public class BallShooterTest extends LinearOpMode{

    DcMotor ballLeftMotor = hardwareMap.dcMotor.get("ball_left");
    DcMotor ballRightMotor = hardwareMap.dcMotor.get("ball_right");
    /*
    final double STOP_POSITION  = 0.5;
    double currTime;
    */

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        //bottom : a; right : b; left: x; top: y
        if(gamepad1.y){
            // < 0.5 is clockwise
            servo2.setPosition(0.2);
            //something needs to be added to the robot to stop the servo from moving the entire 180 degrees
        }
        servo2.setPosition(STOP_POSITION);

        currTime = this.time;

        while(this.time - currTime < 3) {
            motor1.setPower(0.5);
        }

        motor1.setPower(0);
    }
    */

        if(gamepad1.a){
            ballLeftMotor.setPower(0.5); //turn on ball shooting motor
            ballRightMotor.setPower(-0.5);
        } else if(gamepad1.x){
            ballLeftMotor.setPower(0); //stop ball shooting motor
            ballRightMotor.setPower(0);
        } else if(gamepad1.b){
            ballLeftMotor.setPower(-0.5);
            ballRightMotor.setPower(0.5);
        }
    }

    public void stopRobot(){
        ballLeftMotor.setPower(0);
        ballRightMotor.setPower(0);
    }
}
