package org.firstinspires.ftc.teamcode.Modules.Others;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class PTO {

    public static double leftDisengagedPosition=0.12, leftEngagedPosition=0.47;
    public static double rightDisengagedPosition=0.48, rightEngagedPosition=0.17;

    public enum State{

        Engaged , Disengaged;
    }
    public State state;

    public Servo servoLeft , servoRight;

    public PTO()
    {
        servoLeft= Hardware.seh0;
        servoRight=Hardware.seh2;

        state=State.Disengaged;
        servoLeft.setPosition(leftDisengagedPosition);
        servoRight.setPosition(rightDisengagedPosition);
    }
    public void Engage()
    {
        state=State.Engaged;
        servoLeft.setPosition(leftEngagedPosition);
        servoRight.setPosition(rightEngagedPosition);


    }
    public void Disengage()
    {
        state=State.Disengaged;
        servoLeft.setPosition(leftDisengagedPosition);
        servoRight.setPosition(rightDisengagedPosition);
    }
}
