package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(group="zz")
public class BeamBreakTest extends OpMode {
    DigitalChannel breakBeam0, breakBeam1;
    
    @Override
    public void init() {
        breakBeam0 = hardwareMap.get(DigitalChannel.class , "bb0");
        breakBeam1 = hardwareMap.get(DigitalChannel.class , "bb1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("bb0",breakBeam0.getState());
        telemetry.addData("bb1",breakBeam1.getState());
        telemetry.update();
    }
}
