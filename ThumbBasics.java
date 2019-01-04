package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ThumbBasics extends LinearOpMode{

    // todo: write your code here
    private Servo right_thumb;
    private Servo left_thumb;
    public void runOpMode(){
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        waitForStart();
        while(opModeIsActive()){
        right_thumb.setPosition(0.05);
        left_thumb.setPosition(1.00);
        //^closed possision
        //right 
        //smaller number = closer to center
        //left
        //larger number = closer to center
        //.5 for each  = wide open
        //left should be 0.05 more porportianally for symetry
        //so like left -0.05 + right should = 1
        }
    }
}
