package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ShruthiJaganathan on 11/12/16.
 */
@TeleOp(name="Movement: TeleOp", group="Opmode")
public class MovementTest_TeleOp extends OpMode{

    DcMotor rightMotor; // Right side wheels
    DcMotor leftMotor; // Left Side Wheels

    public void init(){
        rightMotor = hardwareMap.dcMotor.get("right_m");
        leftMotor = hardwareMap.dcMotor.get("left_m");
    }


    public void loop(){

        /* This code only moves forward and backward, no turning.*/

        if(gamepad1.right_stick_y == -1 && gamepad1.left_stick_y == -1){
            rightMotor.setPower(-1);
            leftMotor.setPower(1);
        } else if(gamepad1.left_stick_y == 1 && gamepad1.right_stick_y == 1){
            rightMotor.setPower(1);
            leftMotor.setPower(-1);
        }

        /*
        if(gamepad1.right_stick_y == -1){
            if(gamepad1.dpad_left){
                rightMotor.setPower(1);
                leftMotor.setPower(1);
            } else if(gamepad1.dpad_right){
                rightMotor.setPower(-1);
                leftMotor.setPower(-1);
            } else {
                rightMotor.setPower(-1);
                leftMotor.setPower(1);
            }
        }
        /*else if(gamepad1.right_stick_y == 1){
            if(gamepad1.dpad_left){
                rightMotor.setPower(-1);
                leftMotor.setPower(-1);
            } else if(gamepad1.dpad_right){
                rightMotor.setPower(1);
                leftMotor.setPower(1);
                //telemetry.addData("Turn back right", rightMotor.getPower());
            } else {
                rightMotor.setPower(1);
                leftMotor.setPower(-1);
            }
        }
         else {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }

        telemetry.addData("before if block", rightMotor.getPower());
        if(gamepad1.left_stick_y == -1){ //front on the left joystick part
            telemetry.addData("in if block no 1", rightMotor.getPower());
            if(gamepad1.right_stick_y == -1){
                //go forward
                rightMotor.setPower(-1);
                leftMotor.setPower(1);
            } else if(gamepad1.right_stick_y == 1){
                rightMotor.setPower(1);
                leftMotor.setPower(1);
            } else {
                telemetry.addData("Didn't recieve a second command", rightMotor.getPower());
            }
        } else if(gamepad1.right_stick_y == -1){
            telemetry.addData("in if block", rightMotor.getPower());
            if(gamepad1.left_stick_y == 1){
                rightMotor.setPower(-1);
                leftMotor.setPower(-1);
            }else {
                telemetry.addData("Didn't recieve a second command", rightMotor.getPower());
            }
        } else if(gamepad1.right_stick_y == 1 && gamepad1.left_stick_y == 1) {
            telemetry.addData("in if block", rightMotor);
            rightMotor.setPower(1);
            leftMotor.setPower(-1);
        } else {
            telemetry.addData("in else block ", rightMotor.getPower());
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
        telemetry.update();

        if(gamepad1.dpad_up){
            rightMotor.setPower(-1);
            leftMotor.setPower(1);
            telemetry.addData("Right Motor Front", rightMotor.getPower());
            telemetry.addData("Left Motor Front", leftMotor.getPower());
        } else if(gamepad1.dpad_down){
            rightMotor.setPower(1);
            leftMotor.setPower(-1);
            telemetry.addData("Right Motor Back", rightMotor.getPower());
            telemetry.addData("Left Motor Back", leftMotor.getPower());
        } else {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
        telemetry.update();
        */
    }
}
