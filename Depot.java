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

public class Depot extends LinearOpMode
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
                    return turntostart;
               }
               else 
               {  
                //do stuff if you stay in the state 
                lift.setPower(-0.4); 
                 return this;
               }       
            } 
    }
      public class TurnToStart implements StateMachine.State
      {
            @Override
            public void start()
            {
                  encoderStartR = back_right.getCurrentPosition();
            }
            @Override
            public State update()
            {
                String s = ""+back_right.getCurrentPosition();
                telemetry.addData("drive out state",s) ; 
                telemetry.update();
                if (back_right.getCurrentPosition() < encoderStartR + 2000) 
                {
                      back_left.setPower(-0.4);
                      back_right.setPower(0.4);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return driveout;
                }
            }
            private int encoderStartR;
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
                String s = ""+back_left.getCurrentPosition();
                telemetry.addData("drive out state",s) ; 
                telemetry.update();
                if (back_left.getCurrentPosition() < encoderStart + 9750) 
                {
                      back_left.setPower(0.8);
                      back_right.setPower(0.8);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return turn;
                }
            }
            private int encoderStart;
      }
      public class Turn implements StateMachine.State{
          @Override
            public void start()
            {
                  encoderStartL = back_left.getCurrentPosition();
            }
            @Override
            public State update()
            {
                String s = ""+back_left.getCurrentPosition();
                telemetry.addData("drive out state",s) ; 
                telemetry.update();
                if (back_left.getCurrentPosition() < encoderStartL + 2600) 
                {
                      back_left.setPower(0.4);
                      back_right.setPower(-0.4);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return driveout1;
                }
            }
            private int encoderStartL;
      }
      public class DriveOut1 implements StateMachine.State
      {
            @Override
            public void start()
            {
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update()
            {
                String s = ""+back_left.getCurrentPosition();
                telemetry.addData("drive out state",s) ; 
                telemetry.update();
                if (back_left.getCurrentPosition() < encoderStart + 7500) 
                {
                      back_left.setPower(0.8);
                      back_right.setPower(0.8);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return drop;
                }
            }
            private int encoderStart;
      }
      public class Drop implements StateMachine.State{
          
            public void start(){
            right_thumb.setPosition(0.5);
            left_thumb.setPosition(0.5);
            }
            public State update(){
                telemetry.addData("next","") ; 
                telemetry.update();
                return turn1;
            }
      }
      public class Turn1 implements StateMachine.State{
          @Override
            public void start()
            {
                  encoderStartL = back_left.getCurrentPosition();
            }
            @Override
            public State update()
            {
                String s = ""+back_left.getCurrentPosition();
                telemetry.addData("drive out state",s) ; 
                telemetry.update();
                if (back_left.getCurrentPosition() < encoderStartL + 2400) 
                {
                      back_left.setPower(0.4);
                      back_right.setPower(-0.4);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return driveout2;
                }
            }
            private int encoderStartL;
      }
      public class DriveOut2 implements StateMachine.State
      {
            @Override
            public void start()
            {
                  encoderStart = back_left.getCurrentPosition();
            }
            @Override
            public State update()
            {
                String s = ""+back_left.getCurrentPosition();
                telemetry.addData("drive out state",s) ; 
                telemetry.update();
                if (back_left.getCurrentPosition() < encoderStart + 18000) 
                {
                      back_left.setPower(1);
                      back_right.setPower(1);
                      return this;
                }
                else
                {
                      back_left.setPower(0);
                      back_right.setPower(0);
                      return null;
                }
            }
            private int encoderStart;
      }
      public void runOpMode() {
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        arm_extend = hardwareMap.get(DcMotor.class, "arm_extend");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        //color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        
        //lift motor for back lift
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
        onground = new OnGround();
        turntostart = new TurnToStart();
        driveout = new DriveOut();
        driveout1 = new DriveOut1();
        driveout2 = new DriveOut2();
        drop = new Drop();
        turn = new Turn();
        turn1 = new Turn1();
        machine = new StateMachine(driveout);
        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }
    private OnGround onground;
    private TurnToStart turntostart;
    private DriveOut driveout;
    private DriveOut1 driveout1;
    private DriveOut2 driveout2;
    private Drop drop;
    private Turn turn;
    private Turn1 turn1;
    
    public DcMotor arm_lift;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor arm_extend;
    //public ColorSensor color_prox;
    public Servo right_thumb;
    public Servo left_thumb;
    public DcMotor lift;
    
    private StateMachine machine;
}
