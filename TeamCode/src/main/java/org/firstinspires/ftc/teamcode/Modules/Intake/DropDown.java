package org.firstinspires.ftc.teamcode.Modules.Intake;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class DropDown {

    public static boolean ENABLE=true;

    public static boolean reverse=false;

    public static double intakePos[]={0.535 , 0.51 , 0.475 , 0.45 , 0.43};
    public static double repausPos[]={0.51 , 0.48 , 0.46 , 0.44 , 0.41
    };

    public static int index=0;

    public enum State{
        INTAKE(intakePos[index]) , REPAUS(repausPos[index]) , REVERSE(0.4
        );
        double position;
        State(double position)
        {
            this.position=position;
        }
    }
    State state;

    Servo servo;

    public DropDown(State initialState)
    {
        if(ENABLE)
        {
            servo= Hardware.seh4;
        }
        if(reverse)servo.setDirection(Servo.Direction.REVERSE);
        state=initialState;

        servo.setPosition(state.position);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    public void setIndex(int index)
    {
        index=Math.min(4 , index);
        index=Math.max(0 , index);
        this.index=index;
    }
    private void updateState()
    {
        State.INTAKE.position=intakePos[index];
        State.REPAUS.position=repausPos[index];
    }

    private void updateHardware()
    {
        if(servo.getPosition()!=state.position)
        servo.setPosition(state.position);
    }
    public void update()
    {
        if(index<0)index=0;
        if(index>4)index=4;
        updateState();
        updateHardware();
    }
}
