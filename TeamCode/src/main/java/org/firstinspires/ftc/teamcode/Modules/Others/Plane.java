package org.firstinspires.ftc.teamcode.Modules.Others;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Plane {


    public static double openPosition = 0.43   , closedPosition = 0.505;;
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
        state=State.OPEN;
        servo.setPosition(state.position);
    }
}
