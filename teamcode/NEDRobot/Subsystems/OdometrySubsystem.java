package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OdometrySubsystem extends SubsystemBase {

    private Servo LeftOdo,RightOdo,FrontOdo;
    public enum OdoState {
        UP,
        DOWN
    }
    private double odo_up_pos=1;
    private double odo_close_pos=1;

    public OdometrySubsystem(HardwareMap hardwareMap,boolean isAuto){
        this.LeftOdo = hardwareMap.get(Servo.class,"LeftOdo");
        this.RightOdo = hardwareMap.get(Servo.class,"RightOdo");
        this.FrontOdo = hardwareMap.get(Servo.class,"FrontOdo");

        if(isAuto)
        {
            update(OdoState.DOWN);
        }
        else
        {
            update(OdoState.UP);
        }
    }

    public void update(OdoState state)
    {
        switch (state){
            case UP:
                setOdo(odo_up_pos);
                break;
            case DOWN:
                setOdo(odo_close_pos);
                break;
        }
    }
    public void setOdo(double pos)
    {
        LeftOdo.setPosition(pos);
        RightOdo.setPosition(pos);
        FrontOdo.setPosition(pos);
    }
}
