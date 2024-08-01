package org.firstinspires.ftc.teamcode.OpModes.AutoBlue;

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
     boolean isOne=false;


     int whiteTries=0;
     int ticks=0;

     Node currentNode , goToPurple ,releasingPurple , firstWhite_outtakeDown , firstWhite , goToCoridor , goToYellow , goToYellowBackdrop , releasingYellow;
     Node afterYellow , goBeforeStack , goToStack , takeFromFirstStack , goToBackdrop , preparingWhites , dropWhites , alignForWhites2 , goTo7 , goToStack7 , whiteStack , goPark;

    public  boolean init=false;

    public  void run(HardwareMap hardwareMap , Telemetry telemetry)
    {

        if(!init)
        { init=true;
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
         intake=new Intake();
         outtake=new Outtake(Outtake.State.DOWN);
         pto=new PTO();
         hooks=new Hooks();
         topGripper=new TopGripper(TopGripper.State.OPEN);
         bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
         plane=new Plane();
         extendo=new Extendo(Extendo.State.IN);
         bb0 =hardwareMap.get(DigitalChannel.class , "bb0");
         bb1=hardwareMap.get(DigitalChannel.class , "bb1");
         timer=new ElapsedTime();



         goToPurple=new Node("goToPurple");
         releasingPurple = new Node("releasingPurple");
         firstWhite_outtakeDown=new Node("firstWhite_outtakeDown");
         firstWhite=new Node("firstWhite");
         goToCoridor=new Node("goToCoridor");
         goToYellow=new Node("goToYellow");
         goToStack=new Node("goToStack");
         goToYellowBackdrop=new Node("goToYellowBackdrop");
         releasingYellow=new Node("releasingYellow");
         afterYellow=new Node("afterYellow");
         goBeforeStack=new Node("goBeforeStack");
         takeFromFirstStack=new Node("takeFromFirstStack");
         goToBackdrop=new Node("goToBackdrop");
         preparingWhites=new Node("preparingWhites");
         dropWhites=new Node("dropWhites");
         alignForWhites2=new Node("alignForWhites2");
         goTo7=new Node("goTo7");
         goToStack7=new Node("goToStack7");
         whiteStack=new Node("whiteStack");
         goPark=new Node("goPark");

            DropDown.index=4;
            ticks=0;

        goToPurple.addConditions(
                ()-> {driveTrain.setTargetPosition(32.7 , -17.8 , -1); outtake.goPURPLE();
                    Lift.outPosition=80;
                    }
                ,
                ()-> {return driveTrain.inPosition(1.5 , 1.5 , 0.1);}
                ,
                new Node[] {releasingPurple}
        );

        releasingPurple.addConditions(
                ()-> {extendo.setPosition(90); intake.setState(Intake.State.INTAKE); topGripper.setClosed();}
                ,
                ()-> {return topGripper.isClosed();}
                ,
                new Node[]{firstWhite_outtakeDown}
        );

        firstWhite_outtakeDown.addConditions(
                ()-> {outtake.goDOWN(); timer.reset();}
                ,
                ()->{return true;}
                ,
                new Node[]{firstWhite}
        );

        firstWhite.addConditions(
                ()-> {if(timer.seconds()>2.3)intake.setState(Intake.State.REVERSE);
                    else intake.setState(Intake.State.INTAKE);
                    if(timer.seconds()>2.5){intake.setState(Intake.State.INTAKE) ;
                                      intake.decreaseDropDown();
                                      timer.reset();
                                      whiteTries++;
                    }

                    }
                ,
                ()->{

                    return ((!bb0.getState() && !bb1.getState()) || whiteTries==3);}
                ,
                new Node[]{goToCoridor}
        );

        goToCoridor.addConditions(
                ()-> {
                    intake.setState(Intake.State.REPAUS);
                    extendo.setIN();
                    driveTrain.setTargetPosition(26, -19, -Math.PI/2);
                    if (extendo.state == Extendo.State.IN) {

                        intake.setState(Intake.State.REVERSE);
                        topGripper.setOpen();
                        bottomGripper.setOpen();
                    }
                    }
                ,
                ()->    {if(extendo.state==Extendo.State.IN){

                    topGripper.setOpen();
                    bottomGripper.setOpen();
                }
                    return (driveTrain.inPosition(1.5 , 8 , 0.1) && extendo.state==Extendo.State.IN && topGripper.isOpen() && bottomGripper.isOpen());}
                , new Node[]{goToYellow}


        );
        goToYellow.addConditions(
                ()->{
                    driveTrain.setTargetPosition(26 , 70 , -Math.PI/2);
                    intake.setState(Intake.State.REPAUS);
                }
                ,
                ()->{if( driveTrain.inPosition(2 , 20 , 0.35))if(!bb0.getState() && !bb1.getState())intake.decreaseDropDown();

                    return driveTrain.inPosition(2 , 20 , 0.35);}
                ,
                new Node[]{goToYellowBackdrop}
        );

        goToYellowBackdrop.addConditions(
                ()->{
                    Lift.outPosition=280;
                    MecanumDriveTrain.KP=1.5;
                    outtake.goUP();
                    if(!bb0.getState() && !bb1.getState()) Pitch.index=2;
                    driveTrain.setTargetPosition(27.1 , 88 , -1.13);
                }
                ,
                ()->{
                    return (driveTrain.inPosition(4 , 10 , 0.2) && outtake.state==Outtake.State.UP && (Lift.outPosition-outtake.lift.getPosition())<30);
                }
                ,
                new Node[]{releasingYellow}
        );
        releasingYellow.addConditions(
                ()->{
                    topGripper.setClosed();
                    bottomGripper.setClosed();
                }
                ,
                ()->{
                    return (topGripper.isClosed() && bottomGripper.isClosed());
                }
                ,
                new Node[]{afterYellow}
        );
        afterYellow.addConditions(
                ()->{
                    outtake.goDOWN();
                    Pitch.index=1;
                    driveTrain.setTargetPosition(27.1 , 84.9 , -Math.PI/2);
                }
                ,
                ()->{
                    return (driveTrain.inPosition(1 , 5 , 0.1));
                }
                , new Node[]{goToStack}

        );

        goToStack.addConditions(
                ()->{
                    MecanumDriveTrain.KP=3;
                    driveTrain.setTargetPosition(27.1 , 18.6 , -Math.PI/2);
                    if(driveTrain.localizer.getPoseEstimate().getY()<35)
                    { if(extendo.getPosition()<750)
                    extendo.setVelocity(1);
                    else extendo.setVelocity(0.65);}

                    whiteTries=0;
                    isOne=false;
                    timer.reset();
                    ticks=0;

                }
                ,
                ()->{
                   return driveTrain.inPosition();
                }
                ,
                new Node[]{takeFromFirstStack , takeFromFirstStack}
        );
        takeFromFirstStack.addConditions(
                ()->
                {
                    if(timer.seconds()>2.3)intake.setState(Intake.State.REVERSE);
                    else intake.setState(Intake.State.INTAKE);
                        if(timer.seconds()>2.5){intake.setState(Intake.State.INTAKE) ;
                            intake.decreaseDropDown();
                            timer.reset();
                            whiteTries++;
                        }
                       if((!bb0.getState() || !bb1.getState()) && !isOne)
                       {
                           isOne=true;
                           timer.reset();
                           intake.decreaseDropDown();
                       }

                       if(!bb0.getState() && !bb1.getState())
                       {
                           ticks++;
                           intake.setState(Intake.State.REVERSE);
                       }


                }
                ,
                ()->{
                    if(ticks==3)intake.decreaseDropDown();
                    return (ticks==3 || whiteTries==3);}
                    ,
                    new Node[]{goToBackdrop}
        );
        goToBackdrop.addConditions(
                ()->
                {
                    if(extendo.state!=Extendo.State.IN)
                    intake.setState(Intake.State.REVERSE);
                    extendo.setIN();
                    MecanumDriveTrain.KP=1.5;
                    Lift.outPosition=500;

                    driveTrain.setTargetPosition(27.1 , 85 , -Math.PI/2);

                    if(extendo.state==Extendo.State.IN)
                    {
                        topGripper.setOpen();
                        bottomGripper.setOpen();
                        intake.setState(Intake.State.REPAUS);

                    }
                }
                ,
                ()->{return topGripper.isOpen() && bottomGripper.isOpen();}
                ,
              new Node[] {preparingWhites}
        );

        preparingWhites.addConditions(
                ()->{
                    outtake.goUP();
                    isOne=false;
                }
                ,
            ()->{
                    return (outtake.state==Outtake.State.UP && driveTrain.inPosition());
                }
                ,
                new Node[]{dropWhites}
        );
        dropWhites.addConditions(
                ()->{
                    topGripper.setClosed();
                    bottomGripper.setClosed();
                }
                ,
                ()->{
                    return (topGripper.isClosed() && bottomGripper.isClosed());
                }
                ,
                new Node[]{alignForWhites2}
        );
        alignForWhites2.addConditions(
                ()->{
                    outtake.goDOWN();
                    driveTrain.setTargetPosition(27.1 , 84 , -Math.PI/2);
                }
                ,
                ()->{
                    return true;
                }
                ,
                new Node[]{goToStack , goTo7 , goPark}

        );

        goTo7.addConditions(
                ()->{
                    DropDown.index=1;
                    driveTrain.setTargetPosition(26 , 15  , -Math.PI/2);
                }
                ,
                ()->{return driveTrain.inPosition(2 , 10 , 0.2);}
                ,
                new Node[]{goToStack7}
        );
        goToStack7.addConditions(
                ()->{
                    driveTrain.setTargetPosition(26 , 14 , -1.86);
                    whiteTries=0;
                    ticks=0;
                }
                ,
                ()->{return driveTrain.inPosition(2 , 2 , 0.005);}
                ,
                new Node[]{whiteStack}
        );
        whiteStack.addConditions(
                ()->{
                    { if(extendo.getPosition()<550)
                        extendo.setVelocity(1);
                    else extendo.setVelocity(0.35);}

                    if(timer.seconds()>2.3)intake.setState(Intake.State.REVERSE);
                    else intake.setState(Intake.State.INTAKE);
                    if(timer.seconds()>2.5){intake.setState(Intake.State.INTAKE) ;
                        intake.decreaseDropDown();
                        timer.reset();
                        whiteTries++;
                    }
                    if((!bb0.getState() || !bb1.getState()) && !isOne)
                    {
                        isOne=true;
                        timer.reset();
                        intake.decreaseDropDown();
                    }

                    if(!bb0.getState() && !bb1.getState())
                    {
                        ticks++;
                        intake.setState(Intake.State.REVERSE);
                    }
                }
                ,
                ()->{
                   return (whiteTries==3 || ticks==3);
                }
                ,
                new Node[]{goToBackdrop}
        );

        goPark.addConditions(
                ()->{
                    driveTrain.setTargetPosition(20 , 85 , -Math.PI/2);
                }
                ,
                ()->{
                return false;}
                ,
                new Node[]{goPark}
        );



        currentNode=goToPurple;}


        {
            telemetry.addData("X" , driveTrain.localizer.getPoseEstimate().getX());
            telemetry.addData("IMU" , Hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("DROPDOWN" , DropDown.index);
            telemetry.addData("State" , currentNode.name);
            telemetry.addData("isOne" , isOne);

            telemetry.update();

            driveTrain.update();
            outtake.update();
            intake.update();
            extendo.update();
            topGripper.update();
            bottomGripper.update();

            currentNode.run();
            if(currentNode.transition())currentNode=currentNode.next[Math.min( currentNode.next.length-1 , currentNode.index++)];


        }








    }
}
