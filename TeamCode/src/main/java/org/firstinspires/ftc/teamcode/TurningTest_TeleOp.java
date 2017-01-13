package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ShruthiJaganathan on 11/19/16.
 */
@Disabled
@TeleOp(name="Turning: TeleOp", group="Opmode")
public class TurningTest_TeleOp extends OpMode {


    DcMotor rightMotor; // Right side wheels
    DcMotor leftMotor; // Left Side Wheels
    DcMotor sweeper; // Sweeper motor

    public void init(){
        rightMotor = hardwareMap.dcMotor.get("right_m");
        leftMotor = hardwareMap.dcMotor.get("left_m");
        sweeper = hardwareMap.dcMotor.get("sweeper_m");
    }

    @Override
    public void loop() {
        if (gamepad1.right_stick_y == -1) {
            rightMotor.setPower(-1);
        }
        else if (gamepad1.right_stick_y == 1) {
            rightMotor.setPower(1);
        }
        else {
            rightMotor.setPower(0);
        }

        if (gamepad1.left_stick_y == -1) {
            leftMotor.setPower(1);
        }
        else if (gamepad1.left_stick_y == 1) {
            leftMotor.setPower(-1);
        }
        else {
            leftMotor.setPower(0);
        }



        if(gamepad2.dpad_up){
            sweeper.setPower(1); //surgical tubing forward
        } else if(gamepad2.dpad_down) {
            sweeper.setPower(-1); // surgical tubing backward
        } else {
            sweeper.setPower(0); //else: stop surgical tubing
        }



    }
}
