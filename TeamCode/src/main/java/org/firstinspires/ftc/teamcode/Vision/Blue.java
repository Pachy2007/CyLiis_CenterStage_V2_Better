package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Hooks;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Others.Plane;
import org.firstinspires.ftc.teamcode.Modules.Others.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.AutoBlue.BlueMiddle;
import org.firstinspires.ftc.teamcode.OpModes.AutoBlue.BlueRight;
import org.firstinspires.ftc.teamcode.OpModes.BlueLeft;
import org.firstinspires.ftc.teamcode.OpModes.RedLeft;
import org.firstinspires.ftc.teamcode.OpModes.RedMiddle;
import org.firstinspires.ftc.teamcode.OpModes.RedRight;
import org.firstinspires.ftc.teamcode.OpModes.TestTrajectory;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;



@Autonomous(group="a")
public class Blue extends LinearOpMode {



    VisionPortal portal;
    PropDetectionBlueFar processor;

    @Override
    public void runOpMode() throws InterruptedException {


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        Hardware.init(hardwareMap);
        processor = new PropDetectionBlueFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();
        int detectionCase = 2;
        double waitTime=0;

        Hardware.imu.resetYaw();
        PTO pto=new PTO();
        Hooks hooks=new Hooks();
        Intake intake=new Intake();
        Outtake outtake=new Outtake(Outtake.State.DOWN);
        intake.setDropDown(4);
        MecanumDriveTrain.KP=1.5;
        Plane plane=new Plane();

        BlueRight blueRight=new BlueRight();
        BlueMiddle blueMiddle=new BlueMiddle();
        org.firstinspires.ftc.teamcode.OpModes.AutoBlue.BlueLeft blueLeft=new org.firstinspires.ftc.teamcode.OpModes.AutoBlue.BlueLeft();

        BlueLeft.waitTime=waitTime;
        TestTrajectory.waitTime=waitTime;
        while(opModeInInit() && !isStopRequested()){
            detectionCase = processor.detection;

//            robotModules.telemetry(telemetry);
            telemetry.addData("detection", detectionCase);
            telemetry.update();

            intake.update();
            outtake.update();}
        //portal.close();

        waitForStart();
        while(opModeIsActive())
            {

              //  if(detectionCase==2)
               //    TestTrajectory.run(hardwareMap , telemetry);

               //y if(detectionCase==1)
                //  blueRight.run(hardwareMap , telemetry);
                //blueMiddle.run(hardwareMap , telemetry);
                blueRight.run(hardwareMap ,telemetry);
               //if(detectionCase==3)
               //     BlueRight.run(hardwareMap);
            }



    }
}
