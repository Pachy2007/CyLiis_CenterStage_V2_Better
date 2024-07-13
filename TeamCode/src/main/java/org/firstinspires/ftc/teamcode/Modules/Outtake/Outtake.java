package org.firstinspires.ftc.teamcode.Modules.Outtake;

public class Outtake
{
    public boolean useExtension=true;

    public enum State{
        DOWN , PREAPERING_UP, UP,
        PREAPEARING_DOWN_BACKDROP , PREAPERING_DOWN , GOING_DOWN;
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
        angularExtension.setDeployed();
        if(useExtension)extension.setDeployed();
    }
    public void goDOWN()
    {
        if(state==State.DOWN || state==State.GOING_DOWN || state==State.PREAPERING_DOWN || state==State.PREAPEARING_DOWN_BACKDROP)return;

        state=State.PREAPEARING_DOWN_BACKDROP;
        pitch.setMiddle();
        turret.setMiddle();
        angularExtension.setRetracted();
    }

    private void updateState()
    {
        if(useExtension && extension.isRetracted() && state==State.UP)extension.setDeployed();
        switch(state)
        {
            case PREAPERING_UP:
                if(angularExtension.isDeployed() && lift.getPosition()>200)
            {
                state=State.UP;
                pitch.setBackDrop();
                turret.setBackdrop();
            }
                break;
            case PREAPEARING_DOWN_BACKDROP:
            if(angularExtension.isRetracted() && turret.isMiddle() && pitch.isMiddle())
            {
                state=State.PREAPERING_DOWN;
                extension.setRetracted();
                angularExtension.setRetracted();
                lift.state=Lift.State.BEFORE_DOWN;
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
            case DOWN:
                break;
        }
    }


    public void update()
    {
        lift.update();
        angularExtension.update();
        extension.update();
        pitch.update();
        turret.update();
        updateState();


    }
}
