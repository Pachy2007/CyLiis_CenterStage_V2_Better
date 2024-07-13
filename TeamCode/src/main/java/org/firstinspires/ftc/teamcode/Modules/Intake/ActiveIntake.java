package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class ActiveIntake {

    DcMotorEx motor;
    public static boolean reverse=true;

    public enum State {
        REPAUS, INTAKE, REVERSE;
    }
    State state;


    public ActiveIntake(State initialState)
    {
        motor= Hardware.meh3;
        if(reverse)motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    private void updateHardware()
    {
        switch (state)
        {
            case REPAUS:motor.setPower(0);
            break;
            case REVERSE:motor.setPower(-1);
            break;
            case INTAKE:motor.setPower(1);
            break;
        }
    }

    public void update()
    {
        updateHardware();
    }
}
