package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by ShruthiJaganathan on 12/27/16.
 */

@Autonomous(name= "Final Autonomous Red Strategy 1", group = "Autonomous")
public class FinalAutonomousRed extends LinearOpMode{

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    ModernRoboticsI2cGyro gyro;
    DcMotor leftMotor; //Wheels left
    DcMotor rightMotor; //Wheels right

    DcMotor ballLeft; //Ball Shooter left motor
    DcMotor ballRight; //Ball Shooter right motor

    Servo flipper;
    Servo beaconServo;
    ColorSensor cr;



    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        leftMotor = hardwareMap.dcMotor.get("left_m");
        rightMotor = hardwareMap.dcMotor.get("right_m");
        beaconServo = hardwareMap.servo.get("beacon");
        cr = hardwareMap.colorSensor.get("mr");
        ballLeft = hardwareMap.dcMotor.get("ball_left");
        ballRight = hardwareMap.dcMotor.get("ball_right");
        flipper = hardwareMap.servo.get("flipper");


        beaconServo.setPosition(0.5);
        flipper.setPosition(0.5);

        waitForStart();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        gyro.resetZAxisIntegrator();

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Going backwards, and since the right motor has already been reversed, pass in motor speed of -1.

        gyroDrive(-1, 48, 0);
        //shoot the ball
        gyroDrive(1, 12, 0); //move "forward" to make sure we don't run over the center
        gyroTurn(0.1, 90); // turn left 90 based on the front side [+90 due to reverse]

        gyroDrive(-1, 24, 0);
        gyroTurn(0.1, -90); //turn right 90 based on front [-90 due to reverse]
        gyroDrive(-0.1, 24, 0);
        gyroTurn(0.1, 90); //turn left 90 based on front [+90 due to reverse]
        //Vuforia Check
        gyroDrive(-1, 32, 0);
        beaconPressSwitch();
        gyroDrive(-1, 2, 0);


    }

    public void ballShoot() {
        //lift up ball using servo
        //shoot ball
        //stop ball

        flipper.setPosition(0.7);
        ballRight.setPower(-1);
        ballLeft.setPower(1);

        double origTime = this.time;

        while(this.time - origTime <= 5){
            //wait to make sure the ball has been caught and shot out
        }

        flipper.setPosition(0.5);
        ballLeft.setPower(0);
        ballRight.setPower(0);


    }

    public void beaconPressSwitch() throws InterruptedException {

        cr.enableLed(false);

        //Color.RGBToHSV(cr.red() * 8, cr.green() * 8, cr.blue() * 8, hsvValues);
        while (opModeIsActive()) {
            double redV = cr.red();
            double blueV = cr.blue();

            telemetry.addData("Red  ", cr.red());
            telemetry.addData("Green", cr.green());
            telemetry.addData("Blue ", cr.blue());


            if (blueV - redV > 0.5) {
                beaconServo.setPosition(0.3);
                telemetry.addData("This is blue!", cr.blue());
            } else if (redV - blueV > 0.5) {
                beaconServo.setPosition(0.7);
                telemetry.addData("This is red!", cr.red());
            } else {
                beaconServo.setPosition(0.5);
            }

            idle();
        }

    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotor.setPower(leftSpeed);
                rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroTurn (  double speed, double angle)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

}
