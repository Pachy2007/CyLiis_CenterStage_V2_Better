package org.firstinspires.ftc.teamcode.Modules.Others;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class BottomGripper {

    public static double openPos=0 , closedPos=1;

    public static double profileMaxVelocity=20 , profileAcceleration=32;
    public BetterMotionProfile profile=new BetterMotionProfile(profileMaxVelocity , profileAcceleration , profileAcceleration);

    public enum State{
        CLOSED(closedPos) , GOING_CLOSED(closedPos , CLOSED),
        OPEN(openPos) , GOING_OPEN(openPos , OPEN);

        State nextState;
        double position;
        State(double position ,State nextState)
        {
            this.position=position;
            this.nextState=nextState;
        }
        State(double position)
        {
            this.position=position;
            this.nextState=this;
        }
    }
    public State state;

    Servo servo;

    public BottomGripper(State initialState)
    {
        state=initialState;
        servo= Hardware.sch4;

        switch(state)
        {
            case OPEN:profile.setMotion(openPos , openPos , 0);
            break;
            case CLOSED:profile.setMotion(closedPos , closedPos , 0);
            break;
        }

    }

    public void setOpen()
    {
        if(state== State.GOING_OPEN || state== State.OPEN)return;
        state= State.GOING_OPEN;
    }
    public void setClosed()
    {
        if(state== State.GOING_CLOSED || state== State.CLOSED)return;
        state= State.GOING_CLOSED;
    }
    public boolean isClosed()
    {
        if(state== State.CLOSED)return true;
        return false;
    }
    public boolean isOpen()
    {
        if(state== State.OPEN)return true;
        return false;
    }

    private void updateState()
    {
        if(profile.finalPosition!=state.position)profile.setMotion(profile.getPosition() , state.position , 0);
        switch (state)
        {
            case GOING_OPEN:if(profile.getPosition()==profile.finalPosition)state=state.nextState;
            break;
            case GOING_CLOSED:if(profile.getPosition()==profile.finalPosition)state=state.nextState;
            break;
        }
    }

    private void updateHardware()
    {
        servo.setPosition(profile.getPosition());
    }

    public void update()
    {
        State.CLOSED.position=closedPos;
        State.GOING_CLOSED.position=closedPos;

        State.OPEN.position=openPos;
        State.GOING_OPEN.position=openPos;

        profile.update();
        updateState();
        updateHardware();
    }
}
