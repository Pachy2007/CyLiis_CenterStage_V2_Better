package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class PitchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        boolean prev1=false;
        boolean prev2=false;
        Hardware.init(hardwareMap);

        Pitch pitch=new Pitch(Pitch.State.MIDDLE);
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.right_bumper && !prev1)pitch.increaseIndex();
            prev1=gamepad1.right_bumper;

            if(gamepad1.left_bumper && !prev2)pitch.decreaseIndex();
            prev2=gamepad1.left_bumper;

            if(gamepad1.a)pitch.setBackDrop();
            if(gamepad1.b)pitch.setMiddle();

            pitch.update();

            telemetry.addData("STATE" , pitch.state);
            telemetry.update();
        }
    }
}
