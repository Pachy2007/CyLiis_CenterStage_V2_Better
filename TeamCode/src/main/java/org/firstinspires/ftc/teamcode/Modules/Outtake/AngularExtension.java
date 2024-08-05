package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class AngularExtension {

    public static double retractedPos=0.71 , deployedPos=0.08;
    ;

    public static double profileMaxVelocity=20 , profileAcceleration=32;
    public BetterMotionProfile profile=new BetterMotionProfile(profileMaxVelocity , profileAcceleration , profileAcceleration);
    public static boolean reversed=false;
    Servo servo;
    public enum State{
        RETRACTED(retractedPos) , GOING_RETRACTED(RETRACTED , retractedPos),
        DEPLOYED(deployedPos) , GOING_DEPLOYED(DEPLOYED , deployedPos) , PURPLE(0.45);

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
    public AngularExtension(State initialState)
    {
        state=initialState;
        servo=Hardware.sch1;
        if(reversed)servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(state.position);
        profile.setMotion(state.position , state.position , 0);
    }

    public void setRetracted()
    {
        if(profile.finalPosition!=retractedPos || state==State.PURPLE){profile.setMotion(profile.getPosition() , retractedPos , profile.getVelocity());
            state= State.GOING_RETRACTED;}
    }
    public void setDeployed()
    {
        if(profile.finalPosition!=deployedPos){profile.setMotion(profile.getPosition() , deployedPos , profile.getVelocity());
            state= State.GOING_DEPLOYED;}
    }
    public boolean isRetracted()
    {
        if(servo.getPosition()==retractedPos)return true;
        return false;
    }
    public boolean isDeployed()
    {
        if(state== State.DEPLOYED && servo.getPosition()==deployedPos && profile.finalPosition==deployedPos)return true;
        return false;
    }
    private void updateState()
    {
        if(state==State.PURPLE)return;
        State.DEPLOYED.position=deployedPos;
        State.GOING_DEPLOYED.position=deployedPos;

        State.RETRACTED.position=retractedPos;
        State.GOING_RETRACTED.position=deployedPos;

        if((state== State.GOING_DEPLOYED || state== State.DEPLOYED) && profile.getPosition()==profile.finalPosition && profile.finalPosition!=deployedPos)
        {profile.setMotion(profile.getPosition() , deployedPos , profile.getVelocity());
            state= State.GOING_DEPLOYED;}

        if((state== State.GOING_RETRACTED || state== State.RETRACTED) && profile.getPosition()==profile.finalPosition && profile.finalPosition!=retractedPos)
        {profile.setMotion(profile.getPosition() , retractedPos , profile.getVelocity());
            state= State.GOING_RETRACTED;}

        if(profile.finalPosition==profile.getPosition())state=state.nextState;
    }

    private void updateHardware()
    {


        if(state==State.PURPLE){if(state.position!=servo.getPosition())servo.setPosition(state.position);}
        else  {if(profile.getPosition()!=servo.getPosition())servo.setPosition(profile.getPosition());}
    }

    public void update()
    {
        profile.update();
        updateState();
        updateHardware();
    }
}