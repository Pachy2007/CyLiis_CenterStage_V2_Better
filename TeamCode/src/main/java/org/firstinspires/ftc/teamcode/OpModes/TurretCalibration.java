package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class TurretCalibration extends LinearOpMode {
    Turret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap);
        turret=new Turret(Turret.State.MIDDLE);
        Hardware.imu.resetYaw();
        boolean prev=false;
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.a && !prev)turret.switchState();
            prev=gamepad1.a;
            turret.update();
            telemetry.addData("IMU" , Hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("STATE" , turret.state);
            telemetry.update();
        }
    }
}
