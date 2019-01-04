
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;




@Autonomous

public class Forward extends LinearOpMode{
   
    private DcMotor back_left;
    private DcMotor back_right;
    private ColorSensor color_prox;
 

    private VuforiaTrackable relicTemplate;
    // todo: write your code here
    public void runOpMode() {
     back_left = hardwareMap.get(DcMotor.class,"back_left" );
     back_right = hardwareMap.get(DcMotor.class,"back_right" );
     color_prox = hardwareMap.get(ColorSensor.class, "color" ); 
      //wait(500);
     
     waitForStart();
     back_right.setDirection(DcMotor.Direction.REVERSE);
       
     while(opModeIsActive())
       { //back_left.setPower(1);
         //telemetry.addData("Green " , color.green() ) ; 
         //telemetry.update();  
         
         //back_right.setPower(1);
         //telemetry.addData(color_prox.get ) ; 
         //telemetry.update();
         
         } 
         
         
    }
}
