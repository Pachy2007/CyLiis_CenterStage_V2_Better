package org.firstinspires.ftc.teamcode.Modules.Outtake;

public class Outtake
{
    public boolean useExtension=true;

    public enum State{
        DOWN , PREAPERING_UP, UP,
        PREAPEARING_DOWN_BACKDROP , PREAPERING_DOWN , GOING_DOWN , PURPLE;
    }
    public State state;
    public AngularExtension angularExtension;
    public Extension extension;
    public Lift lift;
    public Pitch pitch;
    public Turret turret;

    public Outtake(State state)
    {
        this.state=state;

        angularExtension=new AngularExtension(AngularExtension.State.RETRACTED);
        extension=new Extension(Extension.State.RETRACTED);
        lift=new Lift(Lift.State.GOING_DOWN);
        pitch=new Pitch(Pitch.State.MIDDLE);
        turret=new Turret(Turret.State.MIDDLE);
    }


    public void goUP()
    {
        if(state==State.UP || state==State.PREAPERING_UP)return;

        state=State.PREAPERING_UP;
        lift.setPosition(Lift.outPosition);

    }
    public void goPURPLE()
    {
        if(state==State.UP || state==State.PREAPERING_UP)return;

        state=State.PURPLE;
        angularExtension.state= AngularExtension.State.PURPLE;
        lift.setPosition(Lift.outPosition);
        if(useExtension)extension.setDeployed();
    }
    public void goDOWN()
    {
        if(state==State.DOWN || state==State.GOING_DOWN || state==State.PREAPERING_DOWN || state==State.PREAPEARING_DOWN_BACKDROP)return;


            state=State.PREAPEARING_DOWN_BACKDROP;
        pitch.setMiddle();
        turret.setMiddle();

    }

    private void updateState()
    {
        switch(state)
        {
            case PREAPERING_UP:
                if(lift.getPosition()>100)
                angularExtension.setDeployed();
                if(useExtension)extension.setDeployed();
                if(angularExtension.isDeployed() && extension.isDeployed())
            {
                state=State.UP;
                pitch.setBackDrop();
                turret.setBackdrop();
            }
                break;
            case PREAPEARING_DOWN_BACKDROP:
            if(turret.isMiddle() && pitch.isMiddle())
            {
                extension.setRetracted();
                angularExtension.setRetracted();
                state=State.PREAPERING_DOWN;
                if(Lift.outPosition<150)lift.setDown();

            }
            break;
            case PREAPERING_DOWN:
            if(extension.isRetracted() && angularExtension.isRetracted())
            {
                lift.youReady();
                state=State.GOING_DOWN;
            }
            break;
            case GOING_DOWN:
                if(lift.state==Lift.State.DOWN)state=State.DOWN;
                break;
            case UP:
                break;
            case DOWN:
                break;
        }
    }


    public void update()
    {


        turret.update();
        pitch.update();
        lift.update();

        extension.update();
        angularExtension.update();



        updateState();


    }
}
