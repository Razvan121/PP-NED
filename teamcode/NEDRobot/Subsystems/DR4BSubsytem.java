package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DR4BSubsytem extends SubsystemBase {
    public Motor DR4BMotor;

    private int highLevel;
    private int midLevel;
    private int lowLevel;
    private int homeLevel = 0;

    public boolean slideMoving = false;

    public DR4BSubsytem(Motor DR4BMotor){
        this.DR4BMotor = DR4BMotor;
    }

    public void slideHome(){
        slideMoving = true;
        DR4BMotor.setRunMode(Motor.RunMode.PositionControl);

        DR4BMotor.setTargetPosition(homeLevel);

        DR4BMotor.set(0);;

        DR4BMotor.setPositionTolerance(10);

        while(!DR4BMotor.atTargetPosition()){
            DR4BMotor.set(1);
        }

        DR4BMotor.stopMotor();

    }

}
