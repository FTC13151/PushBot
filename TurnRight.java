package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous

public class TurnRight extends LinearOpMode{

    // todo: write your code here
    private DcMotor back_left;
    private DcMotor back_right;
    //private ColorSensor color;
 

    private VuforiaTrackable relicTemplate;
    // todo: write your code here
    public void runOpMode() {
     back_left = hardwareMap.get(DcMotor.class,"back_left" );
     back_right = hardwareMap.get(DcMotor.class,"back_right" );
     //color = hardwareMap.get(ColorSensor.class, "color" ); 
      //wait(500);
     
     waitForStart();
     //fix  right wheel
     back_right.setDirection(DcMotor.Direction.REVERSE);
     
     while(opModeIsActive()){
        back_left.setPower(1);
         
         back_right.setPower(-1);
         
        } 
         
         
    }
}
//backwards
       // back_left.setPower(-1);
         
       //  back_right.setPower(1);
