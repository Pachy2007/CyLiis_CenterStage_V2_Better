package org.firstinspires.ftc.teamcode.Modules.Others;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class TopGripper {

    public static double openPos=0 , closedPos=0.9;

    public static double profileMaxVelocity=47 , profileAcceleration=50;
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

    public TopGripper(State initialState)
    {
        state=initialState;
        servo= Hardware.sch5;

        switch(state)
        {
            case OPEN:profile.setMotion(openPos , openPos , 0);
            servo.setPosition(openPos);
            break;
            case CLOSED:profile.setMotion(closedPos , closedPos , 0);
            servo.setPosition(closedPos);
            break;
        }

    }

    public void setOpen()
    {
        if(state==State.GOING_OPEN || state==State.OPEN)return;
        state=State.GOING_OPEN;
    }
    public void setClosed()
    {
        if(state==State.GOING_CLOSED || state==State.CLOSED)return;
        state=State.GOING_CLOSED;
    }
    public boolean isClosed()
    {
        if(servo.getPosition()==profile.finalPosition && state==State.CLOSED && servo.getPosition()==closedPos)return true;
        return false;
    }
    public boolean isOpen()
    {
        if(servo.getPosition()==profile.finalPosition && state==State.OPEN && servo.getPosition()==openPos)return true;
        return false;
    }

    public double getPosition()
    {
        return servo.getPosition();
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
        if(servo.getPosition()!=profile.getPosition())
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
