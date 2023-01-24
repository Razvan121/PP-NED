package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem extends SubsystemBase {

    public Servo barRight,barLeft;
    public Servo claw;

    private final ElapsedTime timer;

    public static double claw_close=0.46,claw_open=0.52;
    public static double fourbar_extended=0;
    public static double fourbar_retracted=0;
    public static double fourbar_score=0;


    public enum ClawState{
        OPEN,
        CLOSE;

    }

    public enum FourBarState{
        EXTENDED,
        RETRACTED,
        SCORE;
    }

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto)
    {
        this.barLeft = hardwareMap.get(Servo.class,"barLeft");
        this.barRight = hardwareMap.get(Servo.class,"barRight");
        this.claw = hardwareMap.get(Servo.class,"claw");
        this.timer = new ElapsedTime();
        timer.reset();

    }

    public void update(ClawState state)
    {
        switch (state){
            case OPEN:
                claw.setPosition(claw_open);
                break;
            case CLOSE:
                claw.setPosition(claw_close);
                break;
        }
    }

    public void update(FourBarState state){
        switch (state){
            case SCORE:
                setFourbar(fourbar_score);
                break;
            case EXTENDED:
                setFourbar(fourbar_extended);
                break;
            case RETRACTED:
                setFourbar(fourbar_retracted);
        }
    }

    public void setFourbar(double pos)
    {
        barRight.setPosition(pos);
        barLeft.setPosition(1-pos+0.05);
    }


    public void setFourbarFactor(double factor) {
        double fourbarAddition = -0.01 * factor;
        double barLeftPos = barLeft.getPosition();
        if (!(barLeftPos + fourbarAddition > fourbar_retracted) || !(barLeftPos - fourbarAddition < fourbar_extended)) {
            barLeft.setPosition(barLeftPos + fourbarAddition);
            barRight.setPosition(1 - barLeftPos + fourbarAddition);
        }
    }

    public void resetTimer() {
        timer.reset();
    }


}
