package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Lift {


    public static  double outPosition=200;
    public static double adjustingPos=200;
    public static int nr=0;

    public static double kp=0.0134 , ki=0.000000 , kd=0.00025 , ff1=0.05 , ff2=0;
    public PIDController controller=new PIDController(kp , ki , kd , ff1 , ff2);
    public static double maxVelocity=14000 , acc=18000 , dec=10000;
    public BetterMotionProfile profile=new BetterMotionProfile(maxVelocity , acc , dec);
    public static double inPower=-0.1;

    public static double positionTreshHold=30;

    public enum State{
        OUT(outPosition), GOING_OUT(outPosition , OUT),
        DOWN(0), GOING_DOWN(0 , DOWN), BEFORE_DOWN(adjustingPos , GOING_DOWN);

        State nextState;
        double position;

        State(double position)
        {
            nextState=this;
            this.position=position;
        }
        State(double position , State nextState)
        {
            this.nextState=nextState;
            this.position=position;
        }
    }
    public State state;
    DcMotorEx motor;

    public Lift(State initialState)
    {
        motor= Hardware.meh2;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        state=initialState;
        profile.setMotion(0 , 0 , 0);
    }
    public double getPosition()
    {
        return motor.getCurrentPosition();
    }
    public void setPosition(double position)
    {
        outPosition=position;
        state=State.GOING_OUT;
    }
    public void setDown()
    {
        state=State.BEFORE_DOWN;
    }
    public void youReady()
    {
        state=State.GOING_DOWN;
    }

    private void updateState()
    {
        State.OUT.position=outPosition;
        State.GOING_OUT.position=outPosition;
        if(state==State.GOING_OUT && Math.abs(motor.getCurrentPosition()-outPosition)<positionTreshHold)state=state.nextState;
        if(state==State.GOING_DOWN && motor.getVelocity()<0.05)
        {
            nr++;
            if(nr==25)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                nr=0;
                state=state.nextState;
            }
        }
    }
    private void updateHardware()
    {
        double power=0;
        switch (state)
        {
            case GOING_OUT:
            case OUT:
                outPosition=Math.min(outPosition , 1000);
                if(profile.finalPosition!=outPosition)profile.setMotion(motor.getCurrentPosition() , outPosition , profile.getVelocity());
                 power=controller.calculate(profile.getPosition() , motor.getCurrentPosition());
                motor.setPower(power);
                break;
            case BEFORE_DOWN:
                if(profile.finalPosition!=300)profile.setMotion(motor.getCurrentPosition() , 300 , profile.getVelocity());
                power=controller.calculate(profile.getPosition() , motor.getCurrentPosition());
                motor.setPower(power);
                break;
            case DOWN:
               motor.setPower(inPower);
                break;
            case GOING_DOWN:
                motor.setPower(-0.9);
                break;
        }
    }

    public void update()
    {
        //outPosition=Math.max(200 , outPosition);
        //
        // =Math.min(1000 , outPosition);

        profile.update();

        controller.ff1=ff1;
        controller.ff2=ff2;
        controller.kp=kp;
        controller.ki=ki;
        controller.kd=kd;
        updateState();
        updateHardware();
    }
}
