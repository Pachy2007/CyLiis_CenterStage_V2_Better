package org.firstinspires.ftc.teamcode.OpModes.TeleOps;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    ElapsedTime timer=new ElapsedTime();
    ElapsedTime timer2=new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
         boolean before1=false , before2=false , before3=false , before4=false;
        Intake intake=new Intake();
        Extendo extendo=new Extendo(Extendo.State.RESETTING);
        Outtake outtake=new Outtake(Outtake.State.GOING_DOWN);

        BottomGripper bottomGripper=new BottomGripper(BottomGripper.State.CLOSED);
        TopGripper topGripper=new TopGripper(TopGripper.State.CLOSED);
        MecanumDriveTrain drive=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Plane plane=new Plane();
        Hooks hooks=new Hooks();
        PTO pto=new PTO();
        DigitalChannel bb0 =hardwareMap.get(DigitalChannel.class , "bb0");
        DigitalChannel bb1=hardwareMap.get(DigitalChannel.class , "bb1");

        while(opModeInInit())
        {
            intake.update();
            outtake.update();
            bottomGripper.update();
            topGripper.update();
        }
        timer.startTime();
        intake.setDropDown(0);

        waitForStart();
        boolean okPTO=false;

        while(opModeIsActive())
        {
            if(gamepad1.ps)plane.Zone1();

            if(hooks.isDeploy())
            {if(gamepad1.y && outtake.state== Outtake.State.DOWN && !okPTO)
            {
                switch (pto.state)
                {
                    case Engaged:
                        pto.Disengage();
                        drive.setMode(MecanumDriveTrain.State.DRIVE);
                        extendo.inPower=-0.05;
                        Lift.inPower=-0.1;
                        break;
                    case Disengaged:
                        pto.Engage();
                        drive.setMode(MecanumDriveTrain.State.CLIMB);
                        extendo.inPower=0;
                        Lift.inPower=0;
                        break;
                }
            }}
            else if(  gamepad1.y && gamepad2.y)hooks.Deploy();

            okPTO=gamepad1.y;

            if(!bb0.getState() && !bb1.getState() && timer2.seconds()<0.1)
            {
                gamepad1.rumble(2000);
                gamepad2.rumble(2000);
            }
            else if (bb0.getState() || bb1.getState())timer2.reset();


            if(gamepad1.options)Hardware.imu.resetYaw();
            double heading=-Hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x=gamepad1.left_stick_x;
            double y=-gamepad1.left_stick_y;
            double rotate=-gamepad1.left_trigger+gamepad1.right_trigger;
            drive.setTargetVector(x*Math.cos(heading)-y*Math.sin(heading) , x*Math.sin(heading)+y*Math.cos(heading) , rotate);

            if(gamepad2.left_trigger>0.1) intake.setState(Intake.State.REVERSE);
                else if(gamepad2.right_trigger>0.1)
            {
                if((topGripper.isClosed() && bottomGripper.isClosed()) || outtake.lift.state==Lift.State.OUT || outtake.lift.state==Lift.State.GOING_OUT)
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

                if(topGripper.isOpen() && bottomGripper.isOpen())
                outtake.goUP();

            }
            if(gamepad2.b && outtake.state==Outtake.State.UP)
            {
                outtake.goDOWN();
                topGripper.setClosed();
                bottomGripper.setClosed();
            }
            if(Pitch.index==1)
            {if(gamepad1.y && outtake.state== Outtake.State.UP)
            {
                topGripper.setClosed();
            }
            if(gamepad1.a && outtake.state== Outtake.State.UP)
            {
                bottomGripper.setClosed();
            }
                if(gamepad1.b && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                    topGripper.setClosed();
                }
            }
            if(Pitch.index==2)
            {
                if(gamepad1.b && outtake.state== Outtake.State.UP)
                {
                    topGripper.setClosed();
                }
                if(gamepad1.x && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                }
                if(gamepad1.a && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                    topGripper.setClosed();
                }
            }
            if(Pitch.index==3)
            {
                if(gamepad1.a && outtake.state== Outtake.State.UP)
                {
                    topGripper.setClosed();
                }
                if(gamepad1.y && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                }
                if(gamepad1.b && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                    topGripper.setClosed();
                }
            }
            if(Pitch.index==0)
            {
                if(gamepad1.x && outtake.state== Outtake.State.UP)
                {
                    topGripper.setClosed();
                }
                if(gamepad1.b && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                }
                if(gamepad1.a && outtake.state== Outtake.State.UP)
                {
                    bottomGripper.setClosed();
                    topGripper.setClosed();
                }
            }
            if(gamepad2.x && topGripper.isClosed() && bottomGripper.isClosed())
                outtake.goDOWN();

            if((gamepad2.dpad_up || gamepad1.dpad_up) && !before1)Lift.outPosition+=80;
            before1=gamepad2.dpad_up;
            if(gamepad1.dpad_up)before1=gamepad1.dpad_up;
            if((gamepad2.dpad_down || gamepad1.dpad_down) && !before2)Lift.outPosition-=80;
            before2=gamepad2.dpad_down;
            if(gamepad1.dpad_down)before2=gamepad1.dpad_down;

            if(gamepad1.left_bumper && !before3)outtake.pitch.decreaseIndex();
            before3= gamepad1.left_bumper;
            if(gamepad1.right_bumper && !before4)outtake.pitch.increaseIndex();
            before4=gamepad1.right_bumper;



            if(!(pto.state==PTO.State.Engaged))
            outtake.update();
            extendo.update();
            topGripper.update();
            bottomGripper.update();
            intake.update();
            telemetry.addData("FPS" , 1/timer.seconds());
            telemetry.addData("position" , outtake.extension.getPosition());
            telemetry.addData("positionTurret" , outtake.turret.getPosition());
            telemetry.update();
            timer.reset();
        }
    }
}
