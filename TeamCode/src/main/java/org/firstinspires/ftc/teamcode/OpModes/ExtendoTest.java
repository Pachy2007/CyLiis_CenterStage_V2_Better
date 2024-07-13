package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp(name="EXTENDO TEST")
public class ExtendoTest extends LinearOpMode {

    Extendo extendo;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);

        extendo=new Extendo(Extendo.State.RESETTING);

        waitForStart();

        while(opModeIsActive())
        {
            extendo.setVelocity(-gamepad1.right_stick_y);
            if(gamepad1.a)extendo.setIN();

            extendo.update();
            extendo.telemetry(telemetry);
            telemetry.update();
        }
    }
}
