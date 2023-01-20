package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometrySubsystem {

    private Servo Left,Right,Front;

    public OdometrySubsystem(HardwareMap hardwareMap){
        Left = hardwareMap.get(Servo.class,"Left");
        Right = hardwareMap.get(Servo.class,"Right");
        Front = hardwareMap.get(Servo.class,"Front");
    }

    public void UpOdometry()
    {
        Left.setPosition(1);
        Right.setPosition(1);
        Front.setPosition(1);
    }
    public void DownOdometry()
    {
        Left.setPosition(0);
        Right.setPosition(0);
        Front.setPosition(0);
    }

}
