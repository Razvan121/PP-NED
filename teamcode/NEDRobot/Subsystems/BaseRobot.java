package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class BaseRobot {
    public OdometrySubsystem odometrySubsystem;
    public IntakeSubsystem intakeSubsystem;
    public Dr4bSubsystem dr4bSubsystem;
    public SampleMecanumDrive drive;
    private boolean isAuto = false;

    public BaseRobot(HardwareMap hardwareMap,boolean isAuto) {
        this.isAuto = isAuto;

        //this.odometrySubsystem = new OdometrySubsystem(hardwareMap, isAuto);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, isAuto);
        this.dr4bSubsystem = new Dr4bSubsystem(hardwareMap, isAuto);
        this.drive = new SampleMecanumDrive(hardwareMap);

    }
      public BaseRobot(HardwareMap hardwareMap){
            this(hardwareMap,false);
    }

    public void reset()
    {
        dr4bSubsystem.dr4b_motor.resetEncoder();
    }
    public void read()
    {
        dr4bSubsystem.read();
    }
    public void write()
    {
        dr4bSubsystem.write();
    }


}
