package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ShruthiJaganathan on 12/27/16.
 */

public class FinalAutonomousRedRight extends LinearOpMode{

    ModernRoboticsI2cGyro   gyro        = null;                    // Additional Gyro device
    DcMotor                 leftMotor   = null;
    DcMotor                 rightMotor  = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.1;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    DcMotor ballLeft; //Ball Shooter left motor
    DcMotor ballRight; //Ball Shooter right motor
    DcMotor sweeper;

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
        sweeper = hardwareMap.dcMotor.get("sweeper_m");


        beaconServo.setPosition(0.5);
        flipper.setPosition(0.9);

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating() && opModeIsActive())  {
        }

        gyro.resetZAxisIntegrator();

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Going backwards, and since the right motor has already been reversed, pass in motor speed of -1.

        waitForStart();


        //wait(0.5);
        /*
        sweeper.setPower(1);
        wait(1);
        //wait(1);
        ballShoot();
        sweeper.setPower(0);
        */
        gyroDrive(0.5, 23.5, 0);
        gyroTurn(0.1, 90);
        gyroDrive(0.5, 24, 90);
        gyroTurn(0.1, 0);

        //gyroDrive(0.5, 30, 0);
        //gyroTurn(0.1, 90);
        //gyroDrive(0.5, 11, 90);
        //gyroTurn(0.1, 0);
        wait(0.5);
        ballShoot();

        gyroTurn(0.1, 90);
        gyroDrive(0.5, 24, 90);
        //gyroDrive(0.5, 13, 90);
        gyroTurn(0.1, 0);
        gyroDrive(0.5, 33, 0);
        //gyroDrive(0.5, 21.5, 0);
        gyroTurn(0.1, -90);
        //Vuforia Check
        gyroDrive(0.5, -23.5, -90);
        beaconPressSwitch();
        gyroDrive(0.5, -6, -90);
        gyroDrive(1, 54, -90);

        /*
        gyroTurn(0.1, -90); // turn right 90 based on the front side [+90 due to reverse]
        wait(0.5);
        gyroDrive(1, -24, -90);
        wait(0.5);
        gyroTurn(0.1, -180); //turn right 90 based on front [-90 due to reverse]
        wait(0.5);
        gyroDrive(1, -23, -180);

        wait(0.5);
        gyroTurn(0.1, -90); //turn left 90 based on front [+90 due to reverse]
        */



    }

    public void wait(double seconds) {
        double origTime = this.time;

        while(this.time - origTime <= seconds && opModeIsActive()){
            //wait
        }
    }

    public void ballShoot() {
        //lift up ball using servo
        //shoot ball
        //stop ball

        ballRight.setPower(-1);
        ballLeft.setPower(1);

        wait(1.0);

        flipper.setPosition(0.7);

        wait(0.5);

        flipper.setPosition(0.9);
        ballLeft.setPower(0);
        ballRight.setPower(0);


    }

    public void beaconPressSwitch() {
        cr.enableLed(false); // Turns off color sensor LED. We found this to be more accurate

        telemetry.addData("Beacon Press in method", " ");
        telemetry.update();

        int blueCount = 0; // Number of times the sensor detects blue
        int redCount = 0; // Number of times the sensor detects red
        int globalCount = 0; // Number of times the sensor detected any color (to remove tries where it can't distinguish a color)
        double time = this.time;

        while (globalCount < 5 && this.time - time < 5 && opModeIsActive()) { // 5 valid color tries
            double redV = cr.red();
            double blueV = cr.blue();

            // Telemetry Data
            telemetry.addData("Red  ", cr.red());
            telemetry.addData("Green", cr.green());
            telemetry.addData("Blue ", cr.blue());
            telemetry.update();


            if (blueV - redV > 0.5) { // If blue is significantly more than red, update values
                blueCount++;
                globalCount++;
                telemetry.addData("This is blue!", cr.blue());
            } else if (redV - blueV > 0.5) { // If red is significantly more than blue, update values
                redCount++;
                globalCount++;
                telemetry.addData("This is red!", cr.red());
            } else {
                beaconServo.setPosition(0.5);
            }

            telemetry.update();

            wait(0.2); // wait a little to see if color changes
        }

        if (blueCount > redCount) { // if blue is more than red
            beaconServo.setPosition(0);
        } else { // if red is more than blue
            beaconServo.setPosition(1);
        }

    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

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
            }

            // Stop all motion;

            rightMotor.setPower(0);
            leftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Direction", gyro.getHeading());
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
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

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
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

}
