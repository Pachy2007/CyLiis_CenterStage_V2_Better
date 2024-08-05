package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Others.Plane;

@TeleOp
public class PlaneTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Plane plane=new Plane();
        boolean prev=false;
        waitForStart();

        while (opModeIsActive())
        {

            Plane.State.OPEN.position=Plane.openPosition;
            Plane.State.CLOSED.position=Plane.closedPosition;
            if((gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) && !prev)plane.Zone1();


            if(gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)prev=true;
            else prev=false;
        }
    }
}
