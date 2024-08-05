package org.firstinspires.ftc.teamcode.Modules.Others;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Robot.Hardware;


@Config
public class Plane {


    public static double openPosition = 0.48   , closedPosition = 0.53;
    public Servo servo;

    public enum State{
        OPEN(openPosition), CLOSED(closedPosition);

        public double position;
        public final State nextState;

        State(double position){
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState){
            this.position = position;
            this.nextState = nextState;
        }
    }
    public State state;
    public Plane()
    {
        state=State.CLOSED;
        servo= Hardware.seh1;
        servo.setPosition(state.position);
    }

    public void Zone1()
    {
        switch (state)
        {
            case CLOSED:
                state=State.OPEN;
                servo.setPosition(openPosition);
                break;
            case OPEN:
                state=State.CLOSED;
                servo.setPosition(closedPosition);
                break;
        }

    }
}
