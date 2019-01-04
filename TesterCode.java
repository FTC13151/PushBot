
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;




@Autonomous

public class TesterCode extends LinearOpMode{
   
    private DcMotor motor;

    private ColorSensor color;
 

    private VuforiaTrackable relicTemplate;
    // todo: write your code here
    public void runOpMode() {
     motor = hardwareMap.get(DcMotor.class,"motor" );
     color = hardwareMap.get(ColorSensor.class, "color" ); 
      //wait(500);
       
     while(true )
       { motor.setPower(1);
         telemetry.addData("Green " , color.green() ) ; 
         telemetry.update();  
         } 
         
         
    }
}
