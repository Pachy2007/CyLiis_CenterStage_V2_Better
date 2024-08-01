package org.firstinspires.ftc.teamcode.Modules.Outtake;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Turret {


    public static boolean ENABLE=true;
    public static double profileMaxVelocity=20 , profileAcceleration=32;

    BetterMotionProfile profile=new BetterMotionProfile(profileMaxVelocity , profileAcceleration , profileAcceleration);

    public static boolean reversed=true;
    public static double backdropPosition;
    public enum State{
        MIDDLE(0.55) , GOING_MIDDLE(0.55 , MIDDLE),
        BACKDROP(backdropPosition);

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

    public Turret(State initialState)
    {
        if(!ENABLE)return;
        servo= Hardware.sch3;
        if(reversed)servo.setDirection(Servo.Direction.REVERSE);
        state=initialState;
        profile.setMotion(0.55 , 0.55 ,0);
    }
    public void setBackdrop()
    {
        state=State.BACKDROP;
    }
    public void setMiddle()
    {
        if(state==State.MIDDLE || state==State.GOING_MIDDLE)return;
        state=State.GOING_MIDDLE;
        profile.setMotion(backdropPosition , 0.55 , 0);
    }
    public void switchState()
    {
        switch (state)
        {
            case BACKDROP:
                state=State.GOING_MIDDLE;
                profile.setMotion(backdropPosition , 0.55 , 0);
                break;
            case MIDDLE:
            case GOING_MIDDLE:
                state=State.BACKDROP;
                break;
        }
    }
    public boolean isMiddle()
    {
        if(state==State.MIDDLE && servo.getPosition()==0.55)return true;
        return false;
    }
    public double getPosition()
    {
        return servo.getPosition();
    }

    private void updateState()
    {
        if(state==State.GOING_MIDDLE)
            if(profile.getPosition()==profile.finalPosition)
            state=State.MIDDLE;
        State.BACKDROP.position=backdropPosition;
    }

    private void updateBackdropPosition() {
        double heading = Hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (heading < 0) heading = Math.PI + heading;
        double backdropPosition = 0.55 + (180 * heading - 90 * Math.PI) / (355 * Math.PI);
        this.backdropPosition = backdropPosition;
    }
    private void updateHardware()
    {
        switch(state)
        {
            case MIDDLE:
                servo.setPosition(0.55);
                break;
            case GOING_MIDDLE:
                servo.setPosition(profile.getPosition());
                break;
            case BACKDROP:
                servo.setPosition(backdropPosition);
                break;
        }
    }
    public void update()
    {
        if(!ENABLE)return;
        profile.update();
        updateBackdropPosition();
        updateState();
        updateHardware();
    }
}
