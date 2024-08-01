package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.PropDetectionBlueFar;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Vision Calibration Blue Far")
public class VisionCalibrationBlueFar extends LinearOpMode {
    private VisionPortal portal;
    private PropDetectionBlueFar processor;

    @Override
    public void runOpMode() throws InterruptedException {
        processor = new PropDetectionBlueFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("Detection", processor.detection);
            telemetry.addData("Right value", processor.rightSum);
            telemetry.addData("Middle value", processor.middleSum);
            telemetry.update();
        }

        waitForStart();
    }
}