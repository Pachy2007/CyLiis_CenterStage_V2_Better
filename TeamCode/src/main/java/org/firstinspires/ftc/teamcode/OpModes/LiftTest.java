package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class LiftTest extends LinearOpMode {


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);

        Lift lift=new Lift(Lift.State.GOING_DOWN);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)lift.setPosition(300);
            lift.update();

            telemetry.addData("TargetPosition",Lift.outPosition);
            telemetry.addData("Position" , lift.getPosition());
            telemetry.addData("State" , lift.state);

            telemetry.update();
        }
    }
}
