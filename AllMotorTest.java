
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;




@Autonomous

public class AllMotorTest extends LinearOpMode{
   
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor arm_extend;
    private DcMotor arm_lift;
    //private ColorSensor color;
 

    private VuforiaTrackable relicTemplate;
    // todo: write your code here
    public void runOpMode() {
     back_left = hardwareMap.get(DcMotor.class,"back_left" );
     back_right = hardwareMap.get(DcMotor.class,"back_right" );
     arm_extend = hardwareMap.get(DcMotor.class,"arm_extend");
     arm_lift =  hardwareMap.get(DcMotor.class, "arm_lift");
     //color = hardwareMap.get(ColorSensor.class, "color" ); 
      //wait(500);
     
     waitForStart();
     back_right.setDirection(DcMotor.Direction.REVERSE);
       
     while(opModeIsActive())
       { back_left.setPower(1);
         //telemetry.addData("Green " , color.green() ) ; 
         //telemetry.update();  
         
         back_right.setPower(1);
         //telemetry.addData("Green " , color.green() ) ; 
         //telemetry.update();
         
         } 
         
         
    }
}
