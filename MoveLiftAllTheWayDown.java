/*
Copyright 2017
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS
IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//import apis
//package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.StateMachine;
//import org.firstinspires.ftc.robotcore.external.State;
//import org.firstinspires.ftc.robotcore.external.StateMachine;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
//coment
/**
*hveohovhhvpei
*/
@TeleOp

public class MoveLiftAllTheWayDown extends LinearOpMode {
    
        public class CalibrateGyro implements StateMachine.State {
            @Override
            public void start() {
            }

            @Override
            public State update() {
            
                        return drive;
                }
        }

        /**
         * Controlls all the stuff.
         */
        public class Drive implements StateMachine.State {
            @Override
            public void start() {
                //ball_arm.setPosition(0);
                left_thumb.setPosition(0);
                right_thumb.setPosition(1);
            
                
            }

            @Override
            public State update() {
                   

                 //LIFT MACHANISM
                    //moving up //top encoder value is -1200
                    if(gamepad1.dpad_up)
                    {  
                  
                          lift.setPower(-0.3); 
                          
                      } 
                
                     else if(gamepad1.dpad_down)
                     {
                       
                          
                            lift.setPower(0.3);
                    }
                    else
                    {
                        lift.setPower(0);
                    }
                    
                    
        

                return this;
            }
        }

    @Override

    public void runOpMode() {

            //left side our variable name
            //right l=side string is name on phone/hub
            //basically just names the parts
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        arm_extend = hardwareMap.get(DcMotor.class, "arm_extend");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
       // color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        //ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        //arm_gyro = hardwareMap.get(GyroSensor.class, "arm_gyro");
        //body_gyro = hardwareMap.get(GyroSensor.class, "body_gyro");
        lift = hardwareMap.get(DcMotor.class, "lift");

        calibrateGyro = new CalibrateGyro();
        drive = new Drive();
        machine = new StateMachine(calibrateGyro);

        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//!!!!!!!!!!!!!!WHY ARE THESE REVERSE?!?!?!?!?!
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE); 
        startingEncoderValue = lift.getCurrentPosition(); 

        waitForStart();

        while (opModeIsActive()) { 
            machine.update();
            
        }
    }
    private DcMotor lift;
    private DcMotor arm_lift;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor arm_extend;
 //   private ColorSensor color_prox;
    private Servo right_thumb;
    private Servo left_thumb;
   // private Servo ball_arm;
    //private GyroSensor arm_gyro;
    //private GyroSensor body_gyro;
    private double thumbSpeed;
    private double speedDivisor;
    private CalibrateGyro calibrateGyro;
    private Drive drive;
    private StateMachine machine;
    private int startingEncoderValue; 
}
