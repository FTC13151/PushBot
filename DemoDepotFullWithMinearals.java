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

public class DemoDepotFullWithMinearals extends LinearOpMode{
      //Add States Here
      public class OnGround implements StateMachine.State
      {
          //REPLACE WITH LUCAS AND LANDON'S CODE
            @Override
            public void start()
            {
                  //find non-changing variable
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
      public class Drive2Minearal1 implements StateMachine.State
      {
          //REPLACE WITH LUCAS AND LANDON'S CODE
            @Override
            public void start()
            {
                  //find non-changing variable
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
      public class CheckMinearal1 implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //find non-changing variable
            }
            @Override
            public State update()
            {
                if (true)//is yellow
                {
                      
                      return s1tr;
                }
                else
                {
                      return null; //midb; SWITCH THIS ONCE S1 IS DONE
                }
            }
      }
      public class S1TR implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //find non-changing variable
            }
            @Override
            public State update()
            {
                if (true)//if(desired criteria is met} 
                {
                      
                      return s1d;//null if last but otherwise name of next state object
                }
                else
                {
                      return this;
                }
            }
      }
      public class S1D implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //find non-changing variable
            }
            @Override
            public State update()
            {
                if (true)//if(desired criteria is met} 
                {
                      
                      return s1drop;//null if last but otherwise name of next state object
                }
                else
                {
                      return this;
                }
            }
      }
      public class S1Drop implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //find non-changing variable
            }
            @Override
            public State update()
            {
                if (true)//if(desired criteria is met} 
                {
                      
                      return s1tr1;//null if last but otherwise name of next state object
                }
                else
                {
                      return this;
                }
            }
      }
      public class S1TR1 implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //find non-changing variable
            }
            @Override
            public State update()
            {
                if (true)//if(desired criteria is met} 
                {
                      
                      return s1d1end;//null if last but otherwise name of next state object
                }
                else
                {
                      return this;
                }
            }
      }
      public class S1D1End implements StateMachine.State
      {
            @Override
            public void start()
            {
                  //find non-changing variable
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
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        arm_extend = hardwareMap.get(DcMotor.class, "arm_extend");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        
        //lift motor for back lift
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
        //instantieate states here
        onground = new OnGround();
        drive2minearal1 = new Drive2Minearal1();
        checkminearal1 = new CheckMinearal1();
        s1tr = new S1TR();
        s1d = new S1D();
        s1drop = new S1Drop();
        s1tr1 = new S1TR1();
        s1d1end = new S1D1End();
        
        machine = new StateMachine(onground);
        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }
    //add state instance fields here
    public OnGround onground;
    public Drive2Minearal1 drive2minearal1;
    public CheckMinearal1 checkminearal1;
    public S1TR s1tr;
    public S1D s1d;
    public S1Drop s1drop;
    public S1TR1 s1tr1;
    public S1D1End s1d1end;
    
    public DcMotor arm_lift;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor arm_extend;
    public ColorSensor color_prox;
    public Servo right_thumb;
    public Servo left_thumb;
    public DcMotor lift;
    
    private StateMachine machine;
}
