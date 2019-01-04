//FACINGCRATER
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous 

public class FaceingCrater extends LinearOpMode
{
      public class OnGround implements StateMachine.State 
    {  //extra variables as needed 
            private int startencoder;
            @Override
            public void start() 
            {   
                //stuff you need when the state starts 
                startencoder = lift.getCurrentPosition();
                String s = ""+lift.getCurrentPosition();
                telemetry.addData("lowering ",s) ; 
                telemetry.update();
            }

            @Override
            public State update()
            {
                String s = ""+lift.getCurrentPosition();
                telemetry.addData("lowering ",s) ; 
                telemetry.update();
                 if(lift.getCurrentPosition()<startencoder-3300) 
               { //do stuff if you stay in the state
                    lift.setPower(0);
                    return driveout;
               }
               else 
               {  
                //do stuff if you stay in the state 
                lift.setPower(-0.7); 
                 return this;
               }       
            } 
    }
      public class DriveOut implements StateMachine.State
      {
            @Override
            public void start()
            {
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update()
            {
                telemetry.addData("drive out state","") ; 
                telemetry.update();
                if (back_left.getCurrentPosition() < encoderStart + 5000) 
                {
                      back_left.setPower(0.4);
                      back_right.setPower(0.4);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return turnleft;
                }
            }
            private int encoderStart;
      }
      public class turnLeft implements StateMachine.State{
          public void start() {
                telemetry.addData("turn left state",""); 
                telemetry.update();
                encoderStartL = back_left.getCurrentPosition();
                encoderStartR = back_right.getCurrentPosition();
                
          }

          @Override
          public State update() {
                telemetry.addData("turn left state",""); 
                telemetry.update();
                  // Rotate left
                  //
                  if((Math.abs(back_left.getCurrentPosition()) > encoderStartL + 750)){
                          back_left.setPower(-0.125);
                          back_right.setPower(0.125);
                          return this;
                  } else {
                          back_left.setPower(0);
                          back_right.setPower(0);
                          return turnleft1;
                  }
            }
            
                  private int encoderStartL;
                  private int encoderStartR;
      }
      /**
      public class CheckMinerals implements StateMachine.State{
            private int encoderStart;
            
            private int bvalue;
            @Override
            public void start(){
            encoderStart = back_left.getCurrentPosition();
            bvalue = color_prox.blue();
            }
            @Override
            public State update(){
                  if(bvalue>8 && bvalue<13){
                      //add knock off
                        return turnleft1;
                  }
                  else{
                    while(back_left.getCurrentPosition() < encoderStart + 250){
                            back_left.setPower(0.125);
                            back_right.setPower(-0.125);
                    }
                    return this;
                  }
            }
      }
      */
      public class turnLeft1 implements StateMachine.State{
            public void start() {
                encoderStartL = back_left.getCurrentPosition();
                encoderStartR = back_right.getCurrentPosition();
                
          }

          @Override
          public State update() {
                  // Rotate left
                  if((Math.abs(back_left.getCurrentPosition()) < encoderStartR + 500)){
                          back_left.setPower(-0.125);
                          back_right.setPower(0.125);
                          return this;
                  } else {
                          back_left.setPower(0);
                          back_right.setPower(0);
                          return drivetowall;
                  }
            }
            
                  private int encoderStartL;
                  private int encoderStartR;
      }
      public class DriveToWall implements StateMachine.State{
            @Override
            public void start(){
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update(){
                if (back_left.getCurrentPosition() < encoderStart + 2000) {
                      back_left.setPower(0.25);
                      back_right.setPower(0.25);
                      return this;
                }
                else{
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return turnleft2;
                }
            }
            private int encoderStart;
      }
      public class turnLeft2 implements StateMachine.State{
        public void start() {
                encoderStartL = back_left.getCurrentPosition();
                encoderStartR = back_right.getCurrentPosition();
                
          }

          @Override
          public State update() {
                  // Rotate left
                  if((Math.abs(back_left.getCurrentPosition() )< encoderStartR + 2000)){
                          back_left.setPower(-0.125);
                          back_right.setPower(0.125);
                          return this;
                  } else {
                          back_left.setPower(0);
                          back_right.setPower(0);
                          return drivetobase;
                  }
            }
            
                  private int encoderStartL;
                  private int encoderStartR;
      }
      public class DriveToBase implements StateMachine.State{
            @Override
            public void start(){
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update(){
                if (back_left.getCurrentPosition() < encoderStart + 2000) {
                      back_left.setPower(0.25);
                      back_right.setPower(0.25);
                      return this;
                }
                else{
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return droptoken;
                }
            }
            private int encoderStart;
      }
      public class DropToken implements StateMachine.State{
            @Override
            public void start(){}
            @Override
            public State update(){
                  return turnaway;
            }
      }
      public class turnAway implements StateMachine.State{
            public void start() {
                encoderStartL = back_left.getCurrentPosition();
                encoderStartR = back_right.getCurrentPosition();
                
          }

          @Override
          public State update() {
                  // Rotate left
                  if((Math.abs(back_left.getCurrentPosition()) < encoderStartR + 2000)){
                          back_left.setPower(-0.125);
                          back_right.setPower(0.125);
                          return this;
                  } else {
                          back_left.setPower(0);
                          back_right.setPower(0);
                          return drivetootherside;
                  }
            }
            
                  private int encoderStartL;
                  private int encoderStartR;
      }
      public class DriveToOtherSide implements StateMachine.State{
            @Override
            public void start(){
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update(){
                if (back_left.getCurrentPosition() < encoderStart + 2000) {
                      back_left.setPower(0.25);
                      back_right.setPower(0.25);
                      return this;
                }
                else{
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return turntocrater;
                }
            }
            private int encoderStart;
      }
      public class turnToCrater implements StateMachine.State{
            public void start() {
                encoderStartL = back_left.getCurrentPosition();
                encoderStartR = back_right.getCurrentPosition();
                
          }

          @Override
          public State update() {
                  // Rotate left
                  if((Math.abs(back_left.getCurrentPosition()) < encoderStartR + 2000)){
                          back_left.setPower(-0.125);
                          back_right.setPower(0.125);
                          return this;
                  } else {
                          back_left.setPower(0);
                          back_right.setPower(0);
                          return drivetocrater;
                  }
            }
            
                  private int encoderStartL;
                  private int encoderStartR;
      }
      public class DriveToCrater implements StateMachine.State{
            @Override
            public void start(){
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update(){
                if (back_left.getCurrentPosition() < encoderStart + 2000) {
                      back_left.setPower(0.25);
                      back_right.setPower(0.25);
                      return this;
                }
                else{
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return this;
                }
            }
            private int encoderStart;
      }
      public void runOpMode() {

            //left side our variable name
            //right l=side string is name on phone/hub
            //basically just names the parts
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        arm_extend = hardwareMap.get(DcMotor.class, "arm_extend");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        //ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        //arm_gyro = hardwareMap.get(GyroSensor.class, "arm_gyro");
        //body_gyro = hardwareMap.get(GyroSensor.class, "body_gyro");

        //drive = new Drive();
        
        //calibrateGyro = new CalibrateGyro();

        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//!!!!!!!!!!!!!!WHY ARE THESE REVERSE?!?!?!?!?!
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        //back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
        driveout = new DriveOut();
        turnleft = new turnLeft();
        //checkminerals = new CheckMinerals();
        turnleft1 = new turnLeft1();
        drivetowall = new DriveToWall();
        turnleft2 = new turnLeft2();
        drivetobase = new DriveToBase();
        droptoken = new DropToken();
        turnaway = new turnAway();
        turntocrater = new turnToCrater();
        onground = new OnGround();
        
        
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        drivetootherside = new DriveToOtherSide();
        drivetocrater = new DriveToCrater();
        machine = new StateMachine(onground);
        //telemetry = new Telemetry();
        
        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }
    private DriveOut driveout;
    private turnLeft turnleft;
    //private CheckMinerals checkminerals;
    private turnLeft1 turnleft1;
    private DriveToWall drivetowall;
    private turnLeft2 turnleft2;
    private OnGround onground;
    private DriveToBase drivetobase;
    private DropToken droptoken;
    private turnAway turnaway;
    private turnToCrater turntocrater;
    private DriveToOtherSide drivetootherside;
    private DriveToCrater drivetocrater;
    private DcMotor arm_lift;
    private DcMotor lift;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor arm_extend;
    private ColorSensor color_prox;
    private Servo right_thumb;
    private Servo left_thumb;
   // private Servo ball_arm;
    //private GyroSensor arm_gyro;
    //private GyroSensor body_gyro;
    private double thumbSpeed;
    private double speedDivisor;
    //private Telemetry telemetry;
    //private CalibrateGyro calibrateGyro;
    //private Drive drive;
    private StateMachine machine;
}
