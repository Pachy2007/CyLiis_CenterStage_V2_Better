package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Extendo {

    public static double kp=0.0075 , ki=0.00000006 , kd=0.00065;

    public  PIDController pidController;
    public  boolean ENABLE=true;

    public  double position=100;

    public  int nr;
    public  boolean reset=false;
    public  double power;
    public  double powerTreshHold=0.05;

    public static double targetPosition;
    public  double inPower=-0.05;
    public  double targetTreshHold=30;
    public  double overflowPower=0.1;

    public  double velocityTreshHold=0.1;

    public  boolean motor1Reversed=true , motor2Reversed=true;

    public enum State{
        IN , RESETTING(IN) , GOING_IN(IN),
        OUT , GOING_OUT(OUT),
        LOCK() , GOING_LOCK(LOCK);

        State nextState;

        State(State nextState)
        {
            this.nextState=nextState;
        }
        State()
        {
            this.nextState=this;
        }
    }
    public State state;

    DcMotorEx motor1 , motor2;

    public DcMotorEx encoder;

    public Extendo(State initialState)
    {
        motor1= Hardware.meh0;
        motor2=Hardware.meh1;
        encoder=Hardware.meh0;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(motor1Reversed)motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        if(motor2Reversed)motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        pidController=new PIDController(kp , ki, kd);

        state=initialState;
    }

    public void setVelocity(double power)
    {
        this.power=power;
            if(Math.abs(power)>0.1)
                state=State.GOING_OUT;
    }
    public void setPosition(double position)
    {
        targetPosition=position;
        state=State.GOING_LOCK;
    }
    public void setIN()
    {
        if(state==State.RESETTING || state==State.GOING_IN || state==State.IN)return;
        power=0;
        position=0;
        reset=true;
    }
    public void resetPosition()
    {
        state=State.RESETTING;
        power=0;
        position=0;
    }
    public double getPosition()
    {
        return encoder.getCurrentPosition();
    }


    private void updateState()
    {
        if(reset==true)
        {
            state=State.RESETTING;
            reset=false;
        }
        switch(state)
        {
            case RESETTING:
            case GOING_IN:
                if(Math.abs(encoder.getVelocity())<velocityTreshHold)nr++;
                if(nr>2)
                {
                    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    state=state.nextState;
                    power=0;
                    position=0;
                    nr=0;
                }
                break;

            case GOING_OUT:
                if(Math.abs(power)<velocityTreshHold)
                    state=state.nextState;
                if(power<-velocityTreshHold && encoder.getCurrentPosition()<250)state=State.GOING_IN;

                break;

            case GOING_LOCK:
                if(Math.abs(encoder.getCurrentPosition()-targetPosition)<targetTreshHold)
                    state=state.nextState;
                break;

            case OUT:
                if(encoder.getCurrentPosition()<250)state=State.GOING_IN;
                break;

            case LOCK:
                if(Math.abs(encoder.getCurrentPosition()-targetPosition)>targetTreshHold)
                    state=State.GOING_LOCK;
                break;

            case IN:
                break;
        }
    }

    private void updateHardware()
    {
        switch(state)
        {
            case GOING_IN:
                motor1.setPower(-1);
                motor2.setPower(-1);
                break;
            case RESETTING:
                motor1.setPower(-1);
                motor2.setPower(-1);
                break;
            case GOING_OUT:
                if(encoder.getCurrentPosition()>1350 && power>0)
                {motor1.setPower(overflowPower);
                    motor2.setPower(overflowPower);}
                else
                {motor1.setPower(power);
                motor2.setPower(power);}


                break;
            case IN:
                motor1.setPower(inPower);
                motor2.setPower(inPower);
                break;
            case LOCK:
            case GOING_LOCK:
                double power= pidController.calculate( targetPosition , encoder.getCurrentPosition());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case OUT:
                motor1.setPower(this.power);
                motor2.setPower(this.power);
                break;
        }
    }

    public void update()
    {
        updateState();

        updateHardware();

    }
    public void telemetry(Telemetry telemetry)
    {
        telemetry.addData("Target Position" , targetPosition);
        telemetry.addData("Position" , encoder.getCurrentPosition());
        telemetry.addData("Velocity" , power);
        telemetry.addData("STATE" , state);
        telemetry.addData("MOTOR" , motor1.getVelocity());

        telemetry.update();
    }

}
