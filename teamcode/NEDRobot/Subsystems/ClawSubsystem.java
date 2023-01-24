package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    public Servo claw;
    public double close=0.46,open=0.52;

    public ClawSubsystem(HardwareMap hardwareMap)
    {
       this.claw = hardwareMap.get(Servo.class,"claw");
    }

    public void CLOSE(){
        claw.setPosition(close);
    }

    public void OPEN(){
        claw.setPosition(open);
    }

}
