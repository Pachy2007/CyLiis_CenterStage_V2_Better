package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;


public class BlueLeft {


    public static double waitTime;
    public static void run(HardwareMap hardwareMap , Telemetry telemetry){


        Hardware.init(hardwareMap);
        Hardware.imu.resetYaw();
         MecanumDriveTrain drive=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        Outtake outtake=new Outtake(Outtake.State.DOWN);
        Lift.outPosition=100;
        ElapsedTime timer=new ElapsedTime();
        timer.startTime();
        TopGripper topGripper=new TopGripper(TopGripper.State.OPEN);
        BottomGripper bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
        Extendo extendo=new Extendo(Extendo.State.GOING_IN);
        Intake intake=new Intake();
        intake.setDropDown(4);
        DigitalChannel bb0 =hardwareMap.get(DigitalChannel.class , "bb0");
        DigitalChannel bb1=hardwareMap.get(DigitalChannel.class , "bb1");
        boolean ok1=false;
        boolean ok=false;
        boolean ok2=false;
        boolean ok3=false;
        boolean ok4=false;
        boolean ok5=false;
        boolean nr00=false;
        boolean ok100=false;
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



        ElapsedTime tiemrParking=new ElapsedTime();
        tiemrParking.startTime();

        int nr40=0;

        int nr9=0;
        ElapsedTime timer2=new ElapsedTime();
        int nrFINAL=0;
        intake.setDropDown(4);
        int index=4;


        while(true)
        {
            telemetry.addData("topGripper" , topGripper.getPosition());
            telemetry.addData("BottomGripper" , bottomGripper.getPosition());
            telemetry.update();
            drive.update();

            if(ok==false)
            {
                outtake.goPURPLE();
            drive.setTargetPosition(32 , 1.6 , -Math.PI/2-0.3);
            ok=true;
            }

            if(ok==true  && ok1==false && drive.inPosition() && topGripper.isClosed())
            {
                ElapsedTime timer1=new ElapsedTime();
                timer1.reset();

                extendo.setPosition(815);
                double nr=0;
                double nr1=0;
                while(true)
                {
                    outtake.goDOWN();
                    if(drive.inPosition())
                        intake.setState(Intake.State.INTAKE);

                    drive.update();
                    intake.update();
                    extendo.update();
                    outtake.update();

                    if(!bb0.getState() && !bb1.getState())nr1++;
                    if(nr1==1)  {break;}

                    if(timer1.seconds()>2)
                    {
                        while (timer1.seconds()<2.3)
                        {intake.setState(Intake.State.REVERSE);
                            intake.update();
                        }
                        timer1.reset();
                        intake.decreaseDropDown();
                        nr++;
                    }
                    if(nr==3)  { break;}

                }
            } else if(drive.inPosition() && ok1==false)topGripper.setClosed();

            if(drive.inPosition() && topGripper.isClosed() && ok1==false && ok==true)
            {
                extendo.setIN();
                outtake.goDOWN();
                drive.setTargetPosition(50 , 0 , -Math.PI/2);
                ok1=true;
                intake.setState(Intake.State.REVERSE);
            }            else if(drive.inPosition() && ok1==false)topGripper.setClosed();

            if(ok2==false && drive.inPosition(3 , 10 , 0.2) && ok1==true && extendo.state==Extendo.State.IN && outtake.state== Outtake.State.DOWN && tiemrParking.seconds()>waitTime)
            {
                drive.setTargetPosition(50 , 60 , -Math.PI/2);
                ok2=true;
                topGripper.setOpen();
                bottomGripper.setOpen();
                intake.setState(Intake.State.REPAUS);
                if(!bb0.getState() && !bb1.getState())Pitch.index=3;

            }
            if(ok2==true && drive.inPosition( 10 , 10 , 0.3) && ok3==false && topGripper.isOpen() && bottomGripper.isOpen())
            {
                Lift.outPosition=380;
                drive.setTargetPosition(22.7 , 87 , -Math.PI/2);
                outtake.goUP();
                ok3=true;
            }
            if(ok3==true && drive.inPosition(1.5 , 2 , 0.2) && ok4==false )
            {
                topGripper.setClosed();
                bottomGripper.setClosed();
                ok4=true;
            }
            if(ok4==true && topGripper.isClosed() && bottomGripper.isClosed() && ok5==false)drive.setTargetPosition(23 , 85 , -Math.PI/2);
            if(ok4==true && topGripper.isClosed() && bottomGripper.isClosed() && drive.inPosition()){outtake.goDOWN() ; ok5=true;}

            if(ok14==false)
            {  index=4;
                intake.setDropDown(4);

                while(ok5==true)
                {
                    drive.update();
                    intake.update();
                    timer.reset();
                    topGripper.update();
                    bottomGripper.update();
                    outtake.update();
                    extendo.update();
                    telemetry.addData("topGripper" , topGripper.getPosition());
                    telemetry.addData("BottomGripper" , bottomGripper.getPosition());
                    telemetry.update();

                    if(ok6==false)
                    {
                        drive.setTargetPosition( 51.5 , 80 , -Math.PI/2);
                        ok6=true;
                        if(tiemrParking.seconds()>25)
                        {
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
                    if(ok6==true && drive.inPosition(3.5 , 10 , 1) && ok7==false)
                    {
                        drive.setTargetPosition(51.5 , 19.8 , -Math.PI/2);

                        ok7=true;
                        if(tiemrParking.seconds()>25)
                        {
                            ok6=true;
                            ok7=true;
                            ok8=true;
                            ok9=true;
                            topGripper.setClosed();
                            bottomGripper.setClosed();
                            MecanumDriveTrain.KP=1.5;

                            intake.setState(Intake.State.REPAUS);

                            extendo.setIN();
                            nrFINAL=1;
                            if(bb0.getState() && bb1.getState()){ ok10=true ; ok11=true ; ok12=true; ok13=true;}

                        }
                    }
                    if(ok7==true && ok8==false && drive.localizer.getPoseEstimate().getY()<60){extendo.setVelocity(1);
                        MecanumDriveTrain.KP=6;
                        if(tiemrParking.seconds()>25)
                        {
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
                        if(extendo.encoder.getCurrentPosition()>700)extendo.setVelocity(0.35);

                    }
                    if(ok7==true && drive.inPosition() && ok8==false)
                    {
                        if(extendo.encoder.getCurrentPosition()>700)extendo.setVelocity(0.35);
                        intake.setState(Intake.State.INTAKE);
                        intake.setDropDown(index);
                        ok8=true;
                        timer2.reset();
                        if(tiemrParking.seconds()>25)
                        {
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
                        if(extendo.encoder.getCurrentPosition()>700)extendo.setVelocity(0.35);
                        if(!bb0.getState() && !bb1.getState())
                        {
                            nr40++;
                            if(nr40==2){ok9=true;
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
                        if(tiemrParking.seconds()>25)
                        {
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
                        drive.setTargetPosition( 50 , 91.2 , -Math.PI/2);
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
                    if(ok10==true && drive.localizer.getPoseEstimate().getY()>60 && ok12==false)
                    {
                        drive.setTargetPosition(49 , 91.2 , -Math.PI/2-0.7);
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
                    if(ok12==true && ok11==true && ok13==false && outtake.turret.state==Turret.State.BACKDROP)
                    {
                        topGripper.setClosed();
                        bottomGripper.setClosed();
                        ok13=true;
                    }
                    if(ok13==true && topGripper.isClosed() && bottomGripper.isClosed() && ok14==false)
                    {
                        outtake.goDOWN();
                        ok14=true;
                        drive.setTargetPosition( 52 , 88 , -Math.PI/2);
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

            if(nrFINAL==1)
            {
                intake.setDropDown(4);
                index=4;
                boolean ok1000=false;
                boolean ok1001=false;
                boolean ok1002=false;
                boolean ok1003=false;
                boolean ok1004=false;
                boolean ok1005=false;
                boolean ok1006=false;
                boolean ok1007=false;
                boolean ok1008=false;
                boolean ok1009=false;
                boolean ok1010=false;
                boolean ok1011=false;
                boolean ok1012=false;
                boolean ok1013=false;
                boolean ok1014=false;
                boolean ok1015=false;
                boolean ok1016=false;

                okA=false;
                nr40=0;
                 nr00=false;
                 nr9=0;
                while(true)
                {
                    if(drive.localizer.getPoseEstimate().getX()>49.5)
                    if(ok1000==false)
                    {
                        drive.setTargetPosition(54 , 15 , -Math.PI/2);
                        ok1000=true;
                    }
                    if(ok1000==true && drive.localizer.getPoseEstimate().getY()<30 && ok1001==false)
                    {
                        drive.setTargetPosition(54 , 15 , -Math.PI/2+0.25);
                        ok1001=true;
                    }
                    if(ok1001==true && ok1002==false && drive.inPosition(10 , 5 , 0.2))
                    {
                        extendo.setVelocity(1);
                        ok1002=true;
                    }
                    if(ok1002==true && ok1003==false)
                    {
                        intake.setDropDown(--index);
                        if(extendo.getPosition()>800)extendo.setVelocity(0.4);
                        ok1003=true;
                        intake.setState(Intake.State.INTAKE);
                    }
                    if(ok1003==true)
                    {
                        if(!bb0.getState() && !bb1.getState())
                        {
                            nr40++;
                            if(nr40==2){ok1004=true;
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
                            ok1004=true;
                        }
                    }

                    if(ok1004==true && ok1005==false)
                    {
                        extendo.setIN();
                        drive.setTargetPosition( 50 , 91.2 , -Math.PI/2);
                        intake.setState(Intake.State.REVERSE);
                        ok1005=true;
                    }
                    if(extendo.state==Extendo.State.IN && ok1005==true && ok1006==false)
                    {
                        topGripper.setOpen();
                        bottomGripper.setOpen();
                        drive.setTargetPosition(49 , 91.2 , -Math.PI/2-0.7);
                        ok1006=true;
                    }
                    if(topGripper.isOpen() && bottomGripper.isOpen() && ok1006==true && ok1007==false)
                    {
                        outtake.goUP();
                        ok1007=true;
                    }
                    if(ok1007==true && outtake.turret.state==Turret.State.BACKDROP && ok1008==false)
                    {
                        topGripper.setClosed();
                        bottomGripper.setClosed();
                        ok1008=true;
                    }
                    if(topGripper.isClosed() && bottomGripper.isClosed() && ok1008==true && ok1009==false)
                    {
                        outtake.goDOWN();
                        drive.setTargetPosition(50.5 , 88 , -Math.PI/2);
                    }




                    outtake.update();
                    intake.update();

                    drive.update();
                    extendo.update();
                    topGripper.update();
                    bottomGripper.update();
                }
            }


            outtake.update();
            intake.update();

            extendo.update();
            topGripper.update();
            bottomGripper.update();



        }
    }
}
//x=52.33 , y=17.52 , heading=1.351