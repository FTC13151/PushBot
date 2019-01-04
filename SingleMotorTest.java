package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous

public class SingleMotorTest extends LinearOpMode{

    // todo: write your code here
    private DcMotor back_left;

    private VuforiaTrackable relicTemplate;
    public void runOpMode() {
     
     //waitForStart();
       
     //while(opModeIsActive()){
        back_left.setPower(1);
         
        //} 
         
    }    
}
