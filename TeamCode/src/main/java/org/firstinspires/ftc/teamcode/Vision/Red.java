package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Hooks;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Others.Plane;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.RedLeft;
import org.firstinspires.ftc.teamcode.OpModes.RedMiddle;
import org.firstinspires.ftc.teamcode.OpModes.RedRight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class Red extends LinearOpMode {

    VisionPortal portal;
    PropDetectionRedFar processor;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        processor = new PropDetectionRedFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();
        int detectionCase = 2;
        Plane plane=new Plane();
        Hooks hooks=new Hooks();
        PTO pto=new PTO();
         double waitTime=0;

        Intake intake=new Intake();
        Outtake outtake=new Outtake(Outtake.State.DOWN);
        intake.setDropDown(4);
        MecanumDriveTrain.KP=1.5;
        RedRight.waitTime=waitTime;
        RedLeft.waitTime=waitTime;
        RedMiddle.waitTime=waitTime;

        while (opModeInInit() && !isStopRequested()) {
            detectionCase = processor.detection;

            telemetry.addData("detection", detectionCase);
            telemetry.update();
            intake.update();
            outtake.update();



        }
       // portal.close();
        waitForStart();

        while (opModeIsActive()) {
            if (detectionCase == 1)
                RedLeft.run(hardwareMap);
            if(detectionCase==2)
                RedMiddle.run(hardwareMap , telemetry);
            if(detectionCase==3)
                RedRight.run(hardwareMap , telemetry);
        }

    }
}
