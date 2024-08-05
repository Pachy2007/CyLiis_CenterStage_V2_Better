package org.firstinspires.ftc.teamcode.Modules.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.opencv.core.Mat;

@Config
public class MecanumDriveTrain {



    public enum State{
        DRIVE , PID , CLIMB;
    }
    State state;

    DcMotorEx frontLeft , frontRight;
    DcMotorEx backLeft , backRight;

    public static double targetX , targetY , targetHeading;
    public double error;
    double rotation;

    public static double lateralMultiplier=2;
    public double realHeading;

    public static double kp=0.1 , ki=0.000086 , kd=0.0165;
    public static double KP=1.27 , KI , KD=0.19;
    PIDController controllerX=new PIDController(kp , ki , kd) , controllerY=new PIDController(kp , ki , kd) , controllerHeading=new PIDController(KP , KI , KD);

    public TwoWheelTrackingLocalizer localizer=new TwoWheelTrackingLocalizer();

    public MecanumDriveTrain(State initialState)
    {
        state=initialState;

        frontLeft=Hardware.mch0;
        frontRight=Hardware.mch3;
        backLeft=Hardware.mch1;
        backRight=Hardware.mch2;

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public boolean inPosition()
    {
        if(Math.abs(targetX-localizer.getPoseEstimate().getX())<1.5 && Math.abs(targetY-localizer.getPoseEstimate().getY())<1.5 && error<0.2)return true;
        return false;
    }
    public boolean inPosition( double x , double y , double error)
    {
        if(Math.abs(targetX-localizer.getPoseEstimate().getX())<x && Math.abs(targetY-localizer.getPoseEstimate().getY())<y && this.error<error)return true;
        return false;
    }

    public void setTargetVector(double x , double y , double rx)
    {
        if(state==State.CLIMB)
        {
            x=0;
            rx=0;
            if(y>0.1)y=1;
            if(y<-0.2)y=-0.4;
        }

        x*=lateralMultiplier;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }
    public void setMode(State state)
    {
        this.state=state;
    }
    public void setTargetPosition(double x , double y , double heading)
    {
        targetX=x;
        targetY=y;
        targetHeading=heading;


    }
    public void setTargetPosition(Pose2d position)
    {
        targetX=position.getX();
        targetY=position.getY();
        targetHeading=position.getHeading();


    }


    public void update()
    {
        controllerX.kp=kp;
        controllerY.kp=kp;

        controllerX.ki=ki;
        controllerY.ki=ki;

        controllerX.kd=kd;
        controllerY.kd=kd;

        controllerHeading.kp=KP;
        controllerHeading.ki=KI;
        controllerHeading.kd=KD;



        localizer.update();

        if(state==State.PID)
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double x;
            double y;




            x = controllerX.calculate(targetX, localizer.getPoseEstimate().getX());

            

            y=-controllerY.calculate(targetY , localizer.getPoseEstimate().getY());

            double heading=localizer.getHeading();
            if(heading<0)realHeading=Math.abs(heading);
            else realHeading=2*Math.PI-heading;

             error=targetHeading-realHeading;
            if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));
             rotation= controllerHeading.calculate(error , 0);


            setTargetVector(y*Math.cos(-heading) - x*Math.sin(-heading) , y*Math.sin(-heading)+x*Math.cos(-heading) , rotation);

        }
    }


}
