package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;



public class RedMiddle{


    public static double waitTime;
    public static MecanumDriveTrain drive;
    public static Outtake outtake;
    public static  TopGripper topGripper;
    public static  BottomGripper bottomGripper;
    public static  Extendo extendo;
    public static  Intake intake;
    public static void run(HardwareMap hardwareMap , Telemetry telemetry){


        Hardware.init(hardwareMap);
        Hardware.imu.resetYaw();
        drive=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        outtake=new Outtake(Outtake.State.DOWN);
        Lift.outPosition=100;
        ElapsedTime timer=new ElapsedTime();
        timer.startTime();
        topGripper=new TopGripper(TopGripper.State.OPEN);
        bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
        extendo=new Extendo(Extendo.State.GOING_IN);
        Intake intake=new Intake();
        Pitch.index=1;
        intake.setDropDown(4);
        DigitalChannel bb0 =hardwareMap.get(DigitalChannel.class , "bb0");
        DigitalChannel bb1=hardwareMap.get(DigitalChannel.class , "bb1");
        boolean ok1=false;
        boolean ok=false;
        boolean ok2=false;
        boolean ok3=false;
        boolean ok4=false;
        boolean ok5=false;
        boolean ok6=false;
        boolean ok7=false;
        boolean ok8=false;
        boolean ok9=false;
        boolean ok10=false;
        boolean ok11=false;
        boolean ok12=false;
        boolean ok13=false;
        boolean ok14=false;
        boolean ok15=false;
        boolean ok16=false;
        boolean okA=false;
        ElapsedTime tiemrParking = new ElapsedTime();
        tiemrParking.startTime();
        int nr40=0;
        int index=4;
        boolean nr00=false;
        boolean ok100=false;

        int nr9=0;
        ElapsedTime timer2=new ElapsedTime();
        int nrFINAL=0;
        intake.setDropDown(4);


        while(true)
        {

            telemetry.addData("Extension" , outtake.extension.getPosition());
            telemetry.update();
            drive.update();

            if(ok==false)
            {
                outtake.goPURPLE();
                drive.setTargetPosition(38.7 , 12 , Math.PI/2);
                ok=true;
            }
            if(ok==true  && ok1==false && drive.inPosition() && topGripper.isClosed())
            {
                ElapsedTime timer1=new ElapsedTime();
                timer1.reset();
                double nr=0;
                double nr1=0;
                extendo.setPosition(270);
                while(true)
                {
                    telemetry.addData("Extension" , outtake.extension.getPosition());
                    telemetry.update();
                    outtake.goDOWN();
                    if(drive.inPosition())
                        intake.setState(Intake.State.INTAKE);

                    drive.update();
                    intake.update();

                    extendo.update();
                    outtake.update();


                    if(!bb0.getState() && !bb1.getState())nr1++;
                    if(nr1==3)break;
                    if(timer1.seconds()>2)
                    {
                        while (timer1.seconds()<2.3)
                        {intake.setState(Intake.State.REVERSE);
                            intake.update();}
                        timer1.reset();
                        intake.decreaseDropDown();
                        nr++;
                    }
                    if(nr==3)break;
                }
            } else if(drive.inPosition() && ok1==false)topGripper.setClosed();

            if(drive.inPosition() && topGripper.isClosed() && ok1==false && ok==true)
            {
                extendo.setIN();
                ok1=true;
                drive.setTargetPosition(50 , 13 , Math.PI/2);
                ok1=true;
                intake.setState(Intake.State.REVERSE);
            }            else if(drive.inPosition() && ok1==false)topGripper.setClosed();


            if(ok2==false && drive.inPosition(3 , 10 , 0.2) && ok1==true && extendo.state==Extendo.State.IN && outtake.state==Outtake.State.DOWN  && tiemrParking.seconds()>waitTime)
            {

                drive.setTargetPosition(50 , -60 , Math.PI/2);
                ok2=true;

                topGripper.setOpen();
                bottomGripper.setOpen();
                intake.setState(Intake.State.REPAUS);
                if(!bb0.getState() && !bb1.getState())Pitch.index=3;

            }

            if(ok2==true && drive.inPosition(10 , 10 , 0.3) && ok3==false)
            {
                Lift.outPosition=370;
                drive.setTargetPosition(24.5 , -87.8 , Math.PI/2);
                outtake.goUP();
                ok3=true;
            }
            if(ok3==true && drive.inPosition() && ok4==false)
            {
                topGripper.setClosed();
                bottomGripper.setClosed();
                ok4=true;
            }
            if(ok4==true && topGripper.isClosed() && bottomGripper.isClosed() && ok5==false)
                drive.setTargetPosition(29 , -85 , Math.PI/2);
            if(ok4==true && drive.inPosition() && topGripper.isClosed() && bottomGripper.isClosed())outtake.goDOWN();

            if(ok4==true && topGripper.isClosed() && bottomGripper.isClosed() && drive.inPosition()){outtake.goDOWN() ; ok5=true;}

            if(ok14==false && ok5==true)
            {  index=4;
                intake.setDropDown(4);

                while(ok5==true)
                {
                    telemetry.addData("Extension" , outtake.extension.getPosition());
                    telemetry.update();
                    drive.update();
                    intake.update();
                    timer.reset();
                    topGripper.update();
                    bottomGripper.update();
                    outtake.update();
                    extendo.update();

                    if(ok6==false)
                    {
                        drive.setTargetPosition( 50.5 , -80 , Math.PI/2);
                        ok6=true;
                        if(tiemrParking.seconds()>25)
                        {
                            ok6=true;
                            ok7=true;
                            ok8=true;
                            ok=true;
                            MecanumDriveTrain.KP=1.5;

                            topGripper.setClosed();
                            bottomGripper.setClosed();
                            intake.setState(Intake.State.REPAUS);

                            extendo.setIN();
                            nrFINAL=1;
                            if(bb0.getState() && bb1.getState()){ ok10=true ; ok11=true ; ok12=true; ok13=true;}

                        }
                    }
                    if(ok6==true && drive.inPosition(2 , 10 , 1) && ok7==false)
                    {
                        drive.setTargetPosition(50.5 , -19 , Math.PI/2);
                        if(tiemrParking.seconds()>25){
                            ok6=true;
                            ok7=true;
                            ok8=true;
                            ok=true;
                            topGripper.setClosed();
                            bottomGripper.setClosed();
                            MecanumDriveTrain.KP=1.5;

                            intake.setState(Intake.State.REPAUS);

                            extendo.setIN();
                            nrFINAL=1;
                            if(bb0.getState() && bb1.getState()){ ok10=true ; ok11=true ; ok12=true; ok13=true;}

                        }
                        ok7=true;
                    }
                    if(ok7==true && ok8==false && drive.localizer.getPoseEstimate().getY()>-60){extendo.setVelocity(0.8);
                        if(tiemrParking.seconds()>25){
                            ok6=true;
                            ok7=true;
                            ok8=true;
                            ok9=true;
                            topGripper.setClosed();
                            bottomGripper.setClosed();
                            intake.setState(Intake.State.REPAUS);
                            MecanumDriveTrain.KP=1.5;

                            extendo.setIN();
                            nrFINAL=1;
                            if(bb0.getState() && bb1.getState()){ ok10=true ; ok11=true ; ok12=true; ok13=true;}
                        }
                        MecanumDriveTrain.KP=5.5;
                        if(extendo.encoder.getCurrentPosition()>700)extendo.setVelocity(0.5);
                        if(extendo.encoder.getCurrentPosition()>900)extendo.setVelocity(0.4);

                    }
                    if(ok7==true && drive.inPosition() && ok8==false)
                    {
                        if(extendo.encoder.getCurrentPosition()>700)extendo.setVelocity(0.5);
                        if(extendo.encoder.getCurrentPosition()>900)extendo.setVelocity(0.4);

                        intake.setState(Intake.State.INTAKE);
                        intake.setDropDown(index);
                        ok8=true;
                        timer2.reset();
                        if(tiemrParking.seconds()>25){
                            ok6=true;
                            ok7=true;
                            ok8=true;
                            ok9=true;
                            MecanumDriveTrain.KP=1.5;

                            topGripper.setClosed();
                            bottomGripper.setClosed();
                            intake.setState(Intake.State.REPAUS);

                            extendo.setIN();
                            nrFINAL=1;
                            if(bb0.getState() && bb1.getState()){ ok10=true ; ok11=true ; ok12=true; ok13=true;}

                        }

                    }
                    if(ok8==true && ok9==false)
                    {
                        if(extendo.encoder.getCurrentPosition()>700)extendo.setVelocity(0.5);
                        if(extendo.encoder.getCurrentPosition()>900)extendo.setVelocity(0.4);

                        if(!bb0.getState() && !bb1.getState())
                        {
                            nr40++;
                            if(nr40==4){ok9=true;
                                intake.setDropDown(--index);}

                        }
                        if((!bb0.getState() || !bb1.getState()) && nr00==false)
                        {
                            nr00=true;
                            intake.setDropDown(--index);
                        }

                        if(timer2.seconds()>2.5 && okA==false){intake.setState(Intake.State.REVERSE);
                            intake.setDropDown(--index);
                            okA=true;
                        }
                        if(timer2.seconds()>2.8){
                            okA=false;
                            intake.setState(Intake.State.INTAKE);
                            nr9++;
                            timer2.reset();
                        }
                        if(nr9==3)
                        {
                            ok9=true;
                        }
                        if(tiemrParking.seconds()>25){
                            ok6=true;
                            ok7=true;
                            ok8=true;
                            ok9=true;
                            topGripper.setClosed();
                            bottomGripper.setClosed();
                            intake.setState(Intake.State.REPAUS);

                            MecanumDriveTrain.KP=1.5;


                            extendo.setIN();
                            nrFINAL=1;
                            if(bb0.getState() && bb1.getState()){ ok10=true ; ok11=true ; ok12=true; ok13=true;}
                        }
                    }
                    if(ok9==true && ok10==false)
                    {
                        nr00=false;
                        ok100=false;
                        extendo.setIN();
                        drive.setTargetPosition( 50 , -91.5 , Math.PI/2);
                        ok10=true;
                    }
                    if(ok10==true && ok11==false && drive.localizer.getPoseEstimate().getX()<80)intake.setState(Intake.State.REVERSE);

                    if(ok10==true && extendo.state== Extendo.State.IN && ok11==false)
                    {
                        MecanumDriveTrain.KP=1.5;
                        intake.setState(Intake.State.REPAUS);
                        topGripper.setOpen();
                        bottomGripper.setOpen();
                        ok11=true;
                    }
                    if(ok10==true && drive.localizer.getPoseEstimate().getY()<-60 && ok12==false)
                    {
                        drive.setTargetPosition(48 , -91.5 , Math.PI/2+0.7);
                        Lift.outPosition=650;
                        Pitch.index=1;
                    }
                    if (ok11==true && topGripper.isOpen() && bottomGripper.isOpen() && ok12==false)
                    {
                        Lift.outPosition=600;
                        Pitch.index=1;
                        outtake.goUP();
                        ok12=true;
                    }
                    if(ok12==true && ok11==true && ok13==false && Math.abs(outtake.lift.getPosition()-600)<100)
                    {
                        topGripper.setClosed();
                        bottomGripper.setClosed();
                        ok13=true;
                    }
                    if(ok13==true && topGripper.isClosed() && bottomGripper.isClosed() && ok14==false)
                    {
                        outtake.goDOWN();
                        ok14=true;
                        drive.setTargetPosition( 51 , -84 , Math.PI/2+0.7);
                    }
                    if(ok14==true && nrFINAL==0) {
                        ok6 = false;
                        ok7 = false;
                        ok8 = false;
                        ok9 = false;
                        ok10 = false;
                        ok11 = false;
                        ok12 = false;
                        ok13 = false;
                        ok14 = false;
                        ok15 = false;
                        nr40=0;
                        nr9 = 0;
                        nrFINAL = 1;
                        timer.reset();
                    }
                    if (ok14==true)break;






                }}



            outtake.update();
            intake.update();
            extendo.update();

            topGripper.update();
            bottomGripper.update();

        }


    }


}
