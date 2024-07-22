package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Others.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;


public class BlueLeft {


    public static void run(HardwareMap hardwareMap){


        Hardware.init(hardwareMap);
        Hardware.imu.resetYaw();
         MecanumDriveTrain drive=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        Outtake outtake=new Outtake(Outtake.State.DOWN);
        Lift.outPosition=75;
        ElapsedTime timer=new ElapsedTime();
        timer.startTime();
        TopGripper topGripper=new TopGripper(TopGripper.State.OPEN);
        BottomGripper bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
        Extendo extendo=new Extendo(Extendo.State.GOING_IN);
        boolean ok1=false;
        boolean ok=false;
        boolean ok2=false;
        boolean ok3=false;
        boolean ok4=false;


        while(true)
        {
            drive.update();

            if(ok==false)
            {
                outtake.goPURPLE();
            drive.setTargetPosition(20.86 , 4.84 , -0.846);
            ok=true;
            }

            if(drive.inPosition() && topGripper.isClosed() && ok1==false && ok==true)
            {
                ok1=true;
                outtake.goDOWN();
                drive.setTargetPosition(50 , 0 , -Math.PI/2);
                ok1=true;
            }            else if(drive.inPosition())topGripper.setClosed();

            if(ok2==false && drive.inPosition() && ok1==true)
            {
                drive.setTargetPosition(50 , 60 , -Math.PI/2);
                ok2=true;
                topGripper.setOpen();
                bottomGripper.setOpen();
            }
            if(ok2==true && drive.inPosition() && ok3==false)
            {
                Lift.outPosition=340;
                drive.setTargetPosition(25 , 87.63 , -Math.PI/2);
                outtake.goUP();
                ok3=true;
            }
            if(Math.abs(outtake.lift.getPosition()-Lift.outPosition)<20 && ok3==true && drive.inPosition() && ok4==false)
            {
                topGripper.setClosed();
                bottomGripper.setClosed();
                ok4=true;
            }
            if(ok4==true && topGripper.isClosed() && bottomGripper.isClosed())outtake.goDOWN();



            timer.reset();
            topGripper.update();
            bottomGripper.update();
            outtake.update();
            extendo.update();

        }
    }
}
