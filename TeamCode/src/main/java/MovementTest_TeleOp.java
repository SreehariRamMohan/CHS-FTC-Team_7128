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
        if(gamepad1.dpad_up){
            rightMotor.setPower(0.5);
            leftMotor.setPower(0.5);
        } else if(gamepad1.dpad_down){
            rightMotor.setPower(-0.5);
            leftMotor.setPower(-0.5);
        } else {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }
}
