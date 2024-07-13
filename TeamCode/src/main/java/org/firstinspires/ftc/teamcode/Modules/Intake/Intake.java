package org.firstinspires.ftc.teamcode.Modules.Intake;

public class Intake {


    public enum State{
        REPAUS , INTAKE , REVERSE;
    }
    State state;
    ActiveIntake activeIntake;
    public DropDown dropDown;

    public Intake()
    {
        activeIntake=new ActiveIntake(ActiveIntake.State.REPAUS);
        dropDown=new DropDown(DropDown.State.REPAUS);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    public void increaseDropDown()
    {
        DropDown.index++;
    }

    public void decreaseDropDown()
    {
        DropDown.index--;
    }

    public void setDropDown(int index)
    {
        dropDown.setIndex(index);
    }
    public void update()
    {

        switch (state)
        {
            case REPAUS:
                activeIntake.setState(ActiveIntake.State.REPAUS);
                dropDown.setState(DropDown.State.REPAUS);
                break;
            case INTAKE:
                activeIntake.setState(ActiveIntake.State.INTAKE);
                dropDown.setState(DropDown.State.INTAKE);
                break;
            case REVERSE:
                activeIntake.setState(ActiveIntake.State.REVERSE);
                dropDown.setState(DropDown.State.REVERSE);
                break;
        }
        dropDown.update();
        activeIntake.update();
    }



}
