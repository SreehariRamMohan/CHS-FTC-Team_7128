package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ShruthiJaganathan on 10/16/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BallShooterTestAutonomous extends LinearOpMode{
    Servo flipper = hardwareMap.servo.get("flipper");
    DcMotor shooterLeft = hardwareMap.dcMotor.get("shoot_left");
    DcMotor shooterRight = hardwareMap.dcMotor.get("shoot_right");
    final double STOP_POSITION  = 0.5;
    double currTime = this.time;

    @Override
    public void runOpMode() throws InterruptedException {
        //bottom : a; right : b; left: x; top: y

        while (this.time - currTime < 1.5) {
            // wait
        }

        flipper.setPosition(0.2);

        double currTime = this.time;
        while (this.time - currTime < 1) {
            // wait
        }
        
        flipper.setPosition(STOP_POSITION);

        currTime = this.time;

        while(this.time - currTime < 3) {
            shooterLeft.setPower(0.5);
            shooterRight.setPower(-0.5);
        }

        stopRobot();
    }

    public void stopRobot(){
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        flipper.setPosition(STOP_POSITION);
    }
}
