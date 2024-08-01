package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class ExtensionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Extension extension=new Extension(Extension.State.RETRACTED);
        boolean prevA=false , prevB=false;
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a && !prevA)extension.setDeployed();
            prevA=gamepad1.a;

            if(gamepad1.b && !prevB)extension.setRetracted();
            prevB=gamepad1.b;

            extension.update();


            telemetry.addData("STATE" , extension.state);
            telemetry.update();
        }
    }
}
