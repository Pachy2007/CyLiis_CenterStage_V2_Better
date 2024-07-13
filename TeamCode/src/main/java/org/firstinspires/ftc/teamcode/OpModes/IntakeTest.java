package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp(name="Intake Test")
public class IntakeTest extends LinearOpMode {
    Intake intake;

    Extendo extendo;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    boolean prev1=false , prev2=false;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);

        intake =new Intake();

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.right_bumper && prev1==false)intake.increaseDropDown();
            prev1=gamepad1.right_bumper;
            if(gamepad1.left_bumper && prev2==false)intake.decreaseDropDown();
            prev2=gamepad1.left_bumper;
            if(gamepad1.right_trigger>0.05)intake.setState(Intake.State.INTAKE);
            else if(gamepad1.left_trigger>0.05) intake.setState(Intake.State.REVERSE);
            else intake.setState(Intake.State.REPAUS);
            intake.update();
            telemetry.addData("INDEX" , DropDown.index);
            telemetry.update();
        }
    }
}
