package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ShruthiJaganathan on 11/19/16.
 */

@TeleOp(name="Turning: TeleOp", group="Opmode")
public class TurningTest_TeleOp extends OpMode {

    DcMotor rightMotor; // Right side wheels
    DcMotor leftMotor; // Left Side Wheels

    public void init(){
        rightMotor = hardwareMap.dcMotor.get("right_m");
        leftMotor = hardwareMap.dcMotor.get("left_m");
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
    }
}
