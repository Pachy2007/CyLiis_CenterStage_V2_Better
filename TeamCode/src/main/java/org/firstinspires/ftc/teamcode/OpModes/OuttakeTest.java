package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Outtake outtake=new Outtake(Outtake.State.GOING_DOWN);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)outtake.goUP();
            if(gamepad1.b)outtake.goDOWN();
            outtake.update();

            telemetry.addData("PITCH" , outtake.pitch.state);
            telemetry.addData("ANGULAR" , outtake.angularExtension.state);
            telemetry.addData("TURRET" , outtake.turret.state);
            telemetry.addData("LIFT" , outtake.lift.state);

            telemetry.addData("ALL" , outtake.state);
            telemetry.update();


        }

    }
}
