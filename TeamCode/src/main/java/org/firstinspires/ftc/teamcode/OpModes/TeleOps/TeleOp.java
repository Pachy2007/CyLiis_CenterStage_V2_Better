package org.firstinspires.ftc.teamcode.OpModes.TeleOps;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
         boolean before1=false , before2=false , before3=false , before4=false;
        Intake intake=new Intake();
        Extendo extendo=new Extendo(Extendo.State.RESETTING);
        Outtake outtake=new Outtake(Outtake.State.GOING_DOWN);

        BottomGripper bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
        TopGripper topGripper=new TopGripper(TopGripper.State.CLOSED);

        intake.setDropDown(0);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad2.left_trigger>0.1) intake.setState(Intake.State.REVERSE);
                else if(gamepad2.right_trigger>0.1)
            {
                if(topGripper.isClosed() && bottomGripper.isClosed())
                intake.setState(Intake.State.INTAKE);
                else {
                    topGripper.setClosed();
                    bottomGripper.setClosed();
                }
            }
                else intake.setState(Intake.State.REPAUS);
            if(gamepad2.right_bumper)intake.increaseDropDown();
            if(gamepad2.left_bumper)intake.decreaseDropDown();

            double extendoPower=-gamepad1.right_stick_y;

            if((topGripper.isClosed() && bottomGripper.isClosed()) || (outtake.state!=Outtake.State.DOWN))extendo.setVelocity(extendoPower);
            else if(extendoPower>0)
            {
                topGripper.setClosed();
                bottomGripper.setClosed();
            }
            if(extendo.state== Extendo.State.IN && outtake.state== Outtake.State.DOWN && Math.abs(extendoPower)<0.1 && gamepad2.right_trigger<0.1)
            {
                topGripper.setOpen();
                bottomGripper.setOpen();
            }

            if(gamepad2.x && outtake.state==Outtake.State.DOWN)
            {
                outtake.goUP();
                topGripper.setOpen();
                bottomGripper.setOpen();
            }
            if(gamepad2.b && outtake.state==Outtake.State.UP)
            {
                outtake.goDOWN();
                topGripper.setClosed();
                bottomGripper.setClosed();
            }

            if(gamepad1.y && outtake.state== Outtake.State.UP)
            {
                topGripper.setClosed();
            }
            if(gamepad1.a && outtake.state== Outtake.State.UP)
            {
                bottomGripper.setClosed();
            }
            if(gamepad2.x && topGripper.isClosed() && bottomGripper.isClosed())
                outtake.goDOWN();

            if(gamepad2.dpad_up && !before1)Lift.outPosition+=50;
            before1=gamepad2.dpad_up;

            if(gamepad2.dpad_down && !before2)Lift.outPosition-=50;
            before2=gamepad2.dpad_down;

            if(gamepad1.left_bumper && !before3)outtake.pitch.decreaseIndex();
            before3= gamepad1.left_bumper;
            if(gamepad1.right_bumper && !before4)outtake.pitch.increaseIndex();
            before4=gamepad1.right_bumper;

            outtake.update();
            extendo.update();
            topGripper.update();
            bottomGripper.update();
            intake.update();
        }
    }
}
