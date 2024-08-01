package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Extension {

    public static double retractedPos=0.74 , deployedPos=0.05;

    public static double profileMaxVelocity=20 , profileAcceleration=32;
    public BetterMotionProfile profile=new BetterMotionProfile(profileMaxVelocity , profileAcceleration , profileAcceleration);
    public static boolean reversed=false;
    Servo servo;
    public enum State{
        RETRACTED(retractedPos) , GOING_RETRACTED(RETRACTED , retractedPos),
        DEPLOYED(deployedPos) , GOING_DEPLOYED(DEPLOYED , deployedPos);

        State nextState;
        double position;
        State(double position)
        {
            nextState=this;
            this.position=position;
        }
        State(State nextState , double position)
        {
            this.nextState=nextState;
            this.position=position;
        }

    }
    public State state;
    public Extension(State initialState)
    {
        state=initialState;
        servo=Hardware.sch0;
        if(reversed)servo.setDirection(Servo.Direction.REVERSE);
        profile.setMotion(state.position , state.position , 0);
    }

    public void setRetracted()
    {

        profile.setMotion(profile.getPosition() , retractedPos , profile.getVelocity());
            state=State.GOING_RETRACTED;

    }
    public void setDeployed()
    {
        if(profile.finalPosition!=deployedPos){profile.setMotion(profile.getPosition() , deployedPos , profile.getVelocity());
            state=State.GOING_DEPLOYED;}
    }
    public boolean isRetracted()
    {
        if(servo.getPosition()==profile.finalPosition && state==State.RETRACTED)return true;
        return false;
    }
    public double getPosition()
    {
        return servo.getPosition();
    }
    public boolean isDeployed()
    {
        if(state==State.DEPLOYED)return true;
        return false;
    }
    private void updateState()
    {
        State.DEPLOYED.position=deployedPos;
        State.GOING_DEPLOYED.position=deployedPos;

        State.RETRACTED.position=retractedPos;
        State.GOING_RETRACTED.position=retractedPos;

        if((state==State.GOING_DEPLOYED || state==State.DEPLOYED) && profile.getPosition()==profile.finalPosition && profile.finalPosition!=deployedPos)
        {profile.setMotion(profile.getPosition() , deployedPos , profile.getVelocity());
            state=State.GOING_DEPLOYED;}

        if((state==State.GOING_RETRACTED || state==State.RETRACTED) && profile.getPosition()==profile.finalPosition && profile.finalPosition!=retractedPos)
        {profile.setMotion(profile.getPosition() , retractedPos , profile.getVelocity());
            state=State.GOING_RETRACTED;}

        if(profile.finalPosition==profile.getPosition())state=state.nextState;
    }

    private void updateHardware()
    {

        servo.setPosition(profile.getPosition());
    }

    public void update()
    {
        profile.update();
        updateState();
        updateHardware();

    }
}