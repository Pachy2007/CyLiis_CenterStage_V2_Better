package org.firstinspires.ftc.teamcode.Modules.Others;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Hooks {

    public static double openPos1 = 0.75, openPos2 = 0.25, closedPos1 = 0.5, closedPos2 = 0.5;
    public static double beforeopenPos1 = 0.65, beforeopenPos2 = 0.35;
    BetterMotionProfile profile=new BetterMotionProfile(10 , 10 ,10);
    public Servo servo1 , servo2;

    boolean ok=false;
    public Hooks()
    {
        servo1= Hardware.seh3;
        servo2=Hardware.seh5;

        servo1.setPosition(closedPos1);
        servo2.setPosition(closedPos2);
    }

    public void Deploy()
    {
        servo1.setPosition(openPos1);
        servo2.setPosition(openPos2);

        ok=true;
    }

    public boolean isDeploy()
    {
        if(ok==true)return true;
        else return false;
    }
    public void update()
    {
        profile.update();
    }
}
