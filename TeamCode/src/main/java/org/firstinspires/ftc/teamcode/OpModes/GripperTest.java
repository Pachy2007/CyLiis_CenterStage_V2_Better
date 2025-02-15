package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Others.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class GripperTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);

        TopGripper topGripper=new TopGripper(TopGripper.State.OPEN);
        BottomGripper bottomGripper=new BottomGripper(BottomGripper.State.OPEN);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a){topGripper.setOpen();
                          bottomGripper.setOpen();}
            if(gamepad1.b){topGripper.setClosed();
                          bottomGripper.setClosed();}

            topGripper.update();
            bottomGripper.update();
        }
    }
}
