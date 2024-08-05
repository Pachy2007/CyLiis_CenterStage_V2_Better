package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
@TeleOp
public class TestAuto extends LinearOpMode {
    public static double x ,y , angle;
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        Hardware.init(hardwareMap);

        MecanumDriveTrain drive=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        Extendo extendo=new Extendo(Extendo.State.IN);




        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("X" , drive.localizer.getPoseEstimate().getX());
            telemetry.addData("Y" , drive.localizer.getPoseEstimate().getY());
            telemetry.addData("ANGLE" , drive.localizer.getPoseEstimate().getHeading());

            telemetry.addData("targetX" , x);
            telemetry.addData("targetY" , y);
            telemetry.addData("targetAngle" , angle);

            telemetry.update();


            drive.setTargetPosition(x , y , angle);

            extendo.update();
            drive.update();
        }
    }
}
