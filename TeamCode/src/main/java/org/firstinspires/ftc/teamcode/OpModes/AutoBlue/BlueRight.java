package org.firstinspires.ftc.teamcode.OpModes.AutoBlue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Others.Hooks;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Others.Plane;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.opencv.core.Mat;

public class BlueRight {


    MecanumDriveTrain driveTrain;
    Intake intake;
    Outtake outtake;
    PTO pto;
    Hooks hooks;
    TopGripper topGripper;
    BottomGripper bottomGripper;
    Plane plane;
    Extendo extendo;
    DigitalChannel bb0;
    DigitalChannel bb1;
    ElapsedTime timer;
    boolean isOne = false;


    int whiteTries = 0;
    int ticks = 0;

    Pose2d purplePosition=new Pose2d( 37.189 , -18.7 ,-Math.PI/2-0.1);
    Pose2d coridorPosition=new Pose2d( 51.4 ,-15 , -Math.PI/2);
    Pose2d beforeYellowPosition=new Pose2d(51.4 ,75 , -Math.PI/2);
    Pose2d yellowPosition=new Pose2d(46.5 ,92.1 , -2.27);
    Pose2d beforeGoToStackPosition=new Pose2d( 54 , 83 , -Math.PI/2);
    Pose2d goTo7Position=new Pose2d(53 ,18 , -Math.PI/2);
    Pose2d takeFrom7Position=new Pose2d(53 , 17 , - 1.23);
    Pose2d takeFrom7Position_second=new Pose2d(53 , 17 , - 1.27);
    Pose2d beforeBackdropPosition=new Pose2d( 54 , 80 , -Math.PI/2);
    Pose2d putWhitesPosition=new Pose2d(48.5 , 91 ,-2.25);
    Pose2d takeFrom5Position=new Pose2d(54, 19 , -Math.PI/2);
    Pose2d parkPosition=new Pose2d(51.4 , 86 , - 1.9);

    ElapsedTime timer2=new ElapsedTime();
    boolean ok=true;


    Node currentNode;

    Node goingToPurple , releasePurple , firstWhite , alignForCoridor , beforeYellow , putYellow , beforeGoToStack , goTo7 , takeFrom7 , beforeBackdrop , putWhites , takeFrom5 , park;



    int index = 4;

    public boolean init = false;

    public void run(HardwareMap hardwareMap, Telemetry telemetry) {
        if(ok){

        if(!init)
        {
            timer2.startTime();
            init=true;

            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake();
            outtake=new Outtake(Outtake.State.DOWN);
            pto=new PTO();
            hooks=new Hooks();
            topGripper=new TopGripper(TopGripper.State.OPEN);
            bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
            plane=new Plane();
            extendo=new Extendo(Extendo.State.IN);
            bb0=hardwareMap.get(DigitalChannel.class , "bb0");
            bb1=hardwareMap.get(DigitalChannel.class , "bb1");

            goingToPurple=new Node("goingToPurple");
            releasePurple=new Node("releasePurple");
            firstWhite=new Node("firstWhite");
            alignForCoridor=new Node("alignForCoridor");
            beforeYellow=new Node("beforeYellow");
            putYellow=new Node("putYellow");
            beforeGoToStack=new Node("beforeGoToStack");
            goTo7=new Node("goTo7");
            takeFrom7=new Node("takeFrom7");
            beforeBackdrop=new Node("beforeBackdrop");
            putWhites=new Node("putWhites");
            takeFrom5=new Node("takeFrom5");
            park=new Node("park");
            timer=new ElapsedTime();


            goingToPurple.addConditions(
                    ()-> {driveTrain.setTargetPosition(purplePosition);
                        outtake.goPURPLE();
                        Lift.outPosition=80;
                        DropDown.index=4;
                        }
                            ,
                    ()-> {return driveTrain.inPosition( 2 , 2 , 0.15);}
                            ,
                    new Node[]{releasePurple}
            );
            releasePurple.addConditions(
                    ()->{
                        extendo.setPosition(85);

                        topGripper.setClosed();}
                            ,
                    ()->{
                        if(topGripper.isClosed())
                        {

                            timer.reset();
                            timer.reset();
                            outtake.goDOWN();
                            return true;
                        }
                        return false;
                    }
                            ,
                    new Node[]{firstWhite}
            );
            firstWhite.addConditions(
                    ()->{
                        extendo.setPosition(85);

                        intake.setState(Intake.State.INTAKE);

                        if(timer.seconds()>2.7)
                        {
                            intake.setState(Intake.State.REVERSE);
                        }
                        if(timer.seconds()>3.2)
                        {
                            whiteTries++;
                            intake.setState(Intake.State.INTAKE);
                            DropDown.index--;
                            timer.reset();
                        }

                        if(!bb0.getState() && !bb1.getState())
                        {
                            ticks++;
                            if(ticks==3)intake.setState(Intake.State.REVERSE);
                        }
                    }
                    ,
                    ()->{
                        return (whiteTries==3 || ticks==3);
                    }
                    ,
                    new Node[]{alignForCoridor}
            );
            alignForCoridor.addConditions(
                    ()->{
                        extendo.setIN();
                        driveTrain.setTargetPosition(coridorPosition);


                    }
                    ,
                    ()->{
                        if(driveTrain.inPosition(3 , 10 , 0.2) && extendo.state==Extendo.State.IN)
                            if(!bb0.getState() && !bb1.getState())DropDown.index--;

                        if(extendo.state==Extendo.State.IN)
                        {
                            topGripper.setOpen();
                            bottomGripper.setOpen();
                        }

                        return(driveTrain.inPosition(3 , 10 , 0.2) && extendo.state==Extendo.State.IN);
                    }
                    ,
                    new Node[]{beforeYellow}
            );
            beforeYellow.addConditions(
                    ()-> {
                        intake.setState(Intake.State.REPAUS);
                        driveTrain.setTargetPosition(beforeYellowPosition);
                        Pitch.index=2;
                        Lift.outPosition=350;
                        if(driveTrain.localizer.getPoseEstimate().getY()>50 && topGripper.isOpen() && bottomGripper.isOpen())outtake.goUP();
                    }
                            ,
                    ()->{
                        return driveTrain.inPosition(10 , 10 , 0.2);
                    }
                            ,
                    new Node[]{putYellow}
            );
            putYellow.addConditions(
                    ()->{
                        driveTrain.setTargetPosition(yellowPosition);
                        if(outtake.state== Outtake.State.DOWN)outtake.goUP();
                        if(driveTrain.inPosition(1 , 1 , 0.1) && outtake.lift.getPosition()>310)
                        {
                            topGripper.setClosed();
                            bottomGripper.setClosed();

                        }
                    }
                    ,
                    ()->{
                        return (topGripper.isClosed() && bottomGripper.isClosed());
                    }
                    ,
                    new Node[]{beforeGoToStack}
            );
            beforeGoToStack.addConditions(
                    ()->{
                        if(driveTrain.localizer.getPoseEstimate().getY()<90)
                        outtake.goDOWN();
                    driveTrain.setTargetPosition(beforeGoToStackPosition);}
                    ,
                    ()->{return driveTrain.inPosition(2 , 10 , 0.25);}
                    ,
                    new Node[]{goTo7}
            );
            goTo7.addConditions(
                    ()->{
                        outtake.goDOWN();
                        driveTrain.setTargetPosition(goTo7Position);
                    }
                    ,
                    ()->{
                        timer.reset();
                        ticks=0;
                        if(currentNode.index>1)return true;
                        return driveTrain.inPosition(3 , 12 , 0.3);
                    }
                    ,
                    new Node[]{takeFrom7 , takeFrom7 , takeFrom5 ,takeFrom5}
            );
            takeFrom7.addConditions(
                    ()->{
                        if(currentNode.index==0)
                        driveTrain.setTargetPosition(takeFrom7Position);
                        else driveTrain.setTargetPosition(takeFrom7Position_second);

                        if(driveTrain.inPosition(2, 2 , 0.15))
                        {

                            if(extendo.getPosition()<750){extendo.setVelocity(1); timer.reset(); isOne=false;}
                            else {extendo.setVelocity(0.45);

                            intake.setState(Intake.State.INTAKE);
                                if(timer.seconds()>3)intake.setState(Intake.State.REVERSE);
                                if(timer.seconds()>3.5)
                                {
                                    intake.setState(Intake.State.INTAKE);
                                    intake.decreaseDropDown();
                                    timer.reset();
                                }
                                if((!bb0.getState() || !bb1.getState()) && !isOne)
                                {
                                    isOne=true;
                                    intake.decreaseDropDown();
                                }



                            }

                        }
                        if(!bb1.getState() && !bb0.getState())ticks++;
                    }
                    ,
                    ()->{
                        if(ticks==3)intake.decreaseDropDown();
                        if(ticks==3 && currentNode.index==1)intake.setDropDown(4);

                        return (ticks==3);
                    }
                    ,
                    new Node[]{beforeBackdrop , beforeBackdrop}
            );
            beforeBackdrop.addConditions(
                    ()->{
                        Pitch.index=1;
                        driveTrain.setTargetPosition(beforeBackdropPosition);
                        extendo.setIN();
                        if(extendo.state!=Extendo.State.IN)
                        intake.setState(Intake.State.REVERSE);
                        if(extendo.state==Extendo.State.IN)
                        {
                            topGripper.setOpen();
                            bottomGripper.setOpen();
                        }


                    }
                            ,
                    ()->{
                        return (driveTrain.inPosition(9 , 15 , 0.3) && topGripper.isOpen() && bottomGripper.isOpen());
                    }
                            ,
                    new Node[]{putWhites}
            );
            putWhites.addConditions(
                    ()->{
                        intake.setState(Intake.State.REPAUS);
                        driveTrain.setTargetPosition(putWhitesPosition);
                        if(topGripper.isOpen() && bottomGripper.isOpen())
                        {outtake.goUP();
                        Lift.outPosition=600;}

                        if(driveTrain.inPosition(4 , 1.5 , 0.2) && outtake.lift.getPosition()>350)
                        {
                            topGripper.setClosed();
                            bottomGripper.setClosed();
                        }
                        if(topGripper.isClosed() && bottomGripper.isClosed())
                        {
                            outtake.goDOWN();
                        }
                    }
                            ,
                    ()->{
                        if(topGripper.isClosed() && bottomGripper.isClosed())
                        {
                            outtake.goDOWN();
                        }
                        return (topGripper.isClosed() && bottomGripper.isClosed());}
                            ,
                    new Node[]{beforeGoToStack , beforeGoToStack , beforeGoToStack , park}
            );
            takeFrom5.addConditions(
                    ()->{
                    driveTrain.setTargetPosition(takeFrom5Position);

            if(driveTrain.localizer.getPoseEstimate().getY()<50)
            {
                MecanumDriveTrain.KP=2;
                if(extendo.getPosition()<750){extendo.setVelocity(1); timer.reset(); isOne=false;}
                else {extendo.setVelocity(0.45);

                    intake.setState(Intake.State.INTAKE);
                    if(timer.seconds()>3)intake.setState(Intake.State.REVERSE);
                    if(timer.seconds()>3.5)
                    {
                        intake.setState(Intake.State.INTAKE);
                        intake.decreaseDropDown();
                        timer.reset();
                    }
                    if((!bb0.getState() || !bb1.getState()) && !isOne)
                    {
                        isOne=true;
                        intake.decreaseDropDown();
                    }



                }

            }
                        if(!bb1.getState() && !bb0.getState())ticks++;
            }

                    ,
        ()->{
                        if(ticks==3){intake.decreaseDropDown();
            MecanumDriveTrain.KP=1.27;}

            return (ticks==3);
        }
                ,
                new Node[]{beforeBackdrop}
            );

            park.addConditions(
                    ()->{
                        driveTrain.setTargetPosition(parkPosition);
                    }
                    ,
                    ()->{
                        return false;
                    }
                    ,
                    new Node[]{}
            );




            currentNode=goingToPurple;
        }

        {
            currentNode.run();

            driveTrain.update();
            intake.update();
            extendo.update();
            outtake.update();
            topGripper.update();
            bottomGripper.update();

            if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];



        }}

    }

}

