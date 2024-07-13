package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp(name="DropDown TEST")
public class DropDownTest extends LinearOpMode {
    DropDown dropDown;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);

        dropDown=new DropDown(DropDown.State.REPAUS);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.right_bumper)dropDown.setIndex(dropDown.index++);
            if(gamepad1.left_bumper)dropDown.setIndex(dropDown.index--);
            if(gamepad1.right_trigger>0.05)dropDown.setState(DropDown.State.INTAKE);
            else dropDown.setState(DropDown.State.REPAUS);
            dropDown.update();
        }
    }
}
