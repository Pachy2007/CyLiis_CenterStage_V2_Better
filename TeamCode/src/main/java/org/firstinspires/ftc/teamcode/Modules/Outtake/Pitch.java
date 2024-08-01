package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Pitch {

    public static boolean ENABLE=true;
    public static boolean reversed=false;

    public static double v[]={0.03 , 0.315 , 0.585     , 0.865};
    public static int index=1;

    public static double profileMaxVelocity=20 , profileAcceleration=32;
    public BetterMotionProfile profile=new BetterMotionProfile(profileMaxVelocity , profileAcceleration , profileAcceleration);

    public static double middlePos=0.32;
    public enum State {
        BACKDROP(v[index]),
        MIDDLE(middlePos) , GOING_MIDDLE(middlePos , MIDDLE);

        double position;
        State nextState;

        State(double position)
        {
            this.position=position;
            nextState=this;
        }
        State(double position , State nextState)
        {
            this.position=position;
            this.nextState=nextState;
        }
    }
    public State state;

    Servo servo;

    public Pitch(State initialState)
    {
        if(!ENABLE)return;
        servo= Hardware.sch2;
        if(reversed)servo.setDirection(Servo.Direction.REVERSE);

        state=initialState;
    }

    public void setBackDrop()
    {
        state=State.BACKDROP;
    }
    public void setMiddle()
    {
        if(state==State.GOING_MIDDLE || state==State.MIDDLE)return;
        profile.setMotion(state.position , middlePos , 0);
        state=State.GOING_MIDDLE;
    }
    public boolean isMiddle()
    {
        if(state==State.MIDDLE && servo.getPosition()==middlePos)return true;
        return false;
    }
    public boolean isBackdrop()
    {
        if(state==State.BACKDROP)return true;
        return false;
    }


    public void changeIndex(int index)
    {
        this.index=index;
    }
    public void increaseIndex()
    {
        index=(index+1)%4;
    }
    public void decreaseIndex()
    {
        index--;
        if(index==-1)index=3;
    }


    private void updateState()
    {
        if(state==State.GOING_MIDDLE && profile.finalPosition==profile.getPosition())state=State.MIDDLE;

        State.MIDDLE.position=middlePos;
        State.GOING_MIDDLE.position=profile.getPosition();
        State.BACKDROP.position=v[index];

    }

    private void updateHardware()
    {
        servo.setPosition(state.position);
    }

    public void update()
    {
        if(!ENABLE)return;
        updateState();
        updateHardware();
        profile.update();
    }
}
