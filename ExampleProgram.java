package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.StateMachine.State;

@Autonomous

public class ExampleProgram extends LinearOpMode{

    // todo: write your code here
    public class ExampleState implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //Change Stuff
            }
            @Override
            public State update()
            {
                if (true)//if(desired criteria is met} 
                {
                      
                      return null;//null if last but otherwise name of next state object
                }
                else
                {
                      return this;
                }
            }
      }
      public void runOpMode() {
          //HARDWARE MAP
        HardwareMapMaker hw = new HardwareMapMaker(hardwareMap);
        //OTHER
          
        examplestate = new ExampleState();
        machine = new StateMachine(examplestate);
        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }
    private ExampleState examplestate;
    private StateMachine machine;
}
