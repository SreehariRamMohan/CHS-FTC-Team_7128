package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ShruthiJaganathan on 12/3/16.
 */

@Autonomous(name= "Sweeper Test Auto", group = "Autonomous")
public class SweeperTest_Auto extends LinearOpMode{
    Servo sweeper;
    double origTime = this.time;

    @Override
    public void runOpMode() throws InterruptedException {
        sweeper = hardwareMap.servo.get("sweeper_m");

        waitForStart();

        while(this.time - origTime < 10){

            if(this.time - origTime < 6) {
                sweeper.setPosition(1);
                telemetry.addData("Servo Position: ", sweeper.getPosition());
                telemetry.update();
            }

            if(this.time - origTime >= 6){
                //sweeper.setPosition(0.5);
                telemetry.addData("Stopped: ", sweeper.getPosition());
                telemetry.update();
            }

        }
        //if(this.time - origTime < 6) {

        origTime = this.time;

        while(this.time - origTime < 5)
            sweeper.setPosition(0.5);
            telemetry.addData("Stopped: ", sweeper.getPosition());
            telemetry.update();
        //}
    }
}
