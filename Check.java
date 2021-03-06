
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

public class Check extends LinearOpMode
{
      
      public class CheckMineral implements StateMachine.State{
            private int encoderStart;
            
            private int bvalue;
            @Override
            public void start(){
                encoderStart = back_left.getCurrentPosition();
                bvalue = color_prox.blue();
                telemetry.addData("in start","") ; 
                telemetry.update();
                }
                @Override
            public State update(){
                telemetry.addData("in update", "" + bvalue) ; 
                telemetry.update();
                  if(bvalue>8 && bvalue<13){
                      //add knock off
                      telemetry.addData("if, found","") ; 
                        telemetry.update();
                        return null;
                        
                  }
                  else{
                        telemetry.addData("else","") ; 
                        telemetry.update();
                        int encoderStartL = back_left.getCurrentPosition();
                        while(back_left.getCurrentPosition() < encoderStartL + 1200){
                            telemetry.addData("driveing",""+back_left.getCurrentPosition()) ; 
                            telemetry.update();
                            back_left.setPower(0.4);
                            back_right.setPower(-0.4);
                        }
                        telemetry.addData("stopping","") ; 
                telemetry.update();
                        back_left.setPower(0);
                        back_right.setPower(0);
                        bvalue = color_prox.blue();
                        return this;
                  }
            }
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

        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//!!!!!!!!!!!!!!WHY ARE THESE REVERSE?!?!?!?!?!
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        //back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
        checkmineral = new CheckMineral();
        machine = new StateMachine(checkmineral);
        
        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }
    private CheckMineral checkmineral;
    private DcMotor arm_lift;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor arm_extend;
    private ColorSensor color_prox;
    private Servo right_thumb;
    private Servo left_thumb;
    private double thumbSpeed;
    private double speedDivisor;
    private StateMachine machine;
}
