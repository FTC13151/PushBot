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

public class CraterHangMin extends LinearOpMode{
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
                    return examplestate;
               }
               else 
               {  
                //do stuff if you stay in the state 
                lift.setPower(-0.7); 
                 return this;
               }       
            } 
    }
    // todo: write your code here
    public class ExampleState implements StateMachine.State
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
                if (back_left.getCurrentPosition() < encoderStart + 4000) 
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
          //HARDWARE MAP
         
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        arm_extend = hardwareMap.get(DcMotor.class, "arm_extend");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        
        //lift motor for back lift
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        //OTHER
          
        examplestate = new ExampleState();
        onground = new OnGround();
        machine = new StateMachine(examplestate);
        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }
    private ExampleState examplestate;
    private StateMachine machine;
    private OnGround onground;
    
    public DcMotor arm_lift;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor arm_extend;
    public Servo right_thumb;
    public Servo left_thumb;
    public DcMotor lift;
}
