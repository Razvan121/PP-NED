package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    public Servo claw;
    public Servo intake1;
    public Servo intake2;


    private double closePos=0.44;
    private double openPos=0.52;


    public double intake1TransitionIntake_Pos = 0.275;
    public double intake2TransitionIntake_Pos = 0.119;

    public double intake1Intake_Pos=0.12;
    public double intake2Intake_Pos=0.094;

    public double intake1TransitionDeposit_Pos=0.51;
    public double intake2TransitionDeposit_Pos=0.152;

    public double intake1Deposit_Pos=0.65;
    public double intake2Deposit_Pos=0.171;

    public double intake1Junction_Pos=0.74;
    public double intake2Junction_Pos=0.189;



    public enum ClawState {
        CLOSE,
        OPEN,
    }
    public enum FourbarState{
        INTAKE,
        TRANSITION_INTAKE,
        TRANSITION_DEPOSIT,
        DEPOSIT,
        JUNCTION

    }

    public IntakeSubsystem(HardwareMap hardwareMap,boolean isAuto) {
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.intake1 = hardwareMap.get(Servo.class, "intake1");
        this.intake2 = hardwareMap.get(Servo.class, "intake2");
        this.intake1.setDirection(Servo.Direction.REVERSE);

        if(isAuto)
        {
            update(ClawState.CLOSE);
            update(FourbarState.TRANSITION_INTAKE);
        }
        else
        {
            update(ClawState.CLOSE);
            update(FourbarState.TRANSITION_INTAKE);
        }
    }
    public void update(ClawState state)
    {
        switch (state){
            case CLOSE:
                claw.setPosition(closePos);
                break;
            case OPEN:
                claw.setPosition(openPos);
                break;
        }
    }
    public void update(FourbarState state)
    {
        switch (state){
            case INTAKE:
                setFourbar(intake1Intake_Pos,intake2Intake_Pos);
                break;
            case TRANSITION_INTAKE:
                setFourbar(intake1TransitionIntake_Pos,intake2TransitionIntake_Pos);
                break;
            case TRANSITION_DEPOSIT:
                setFourbar(intake1TransitionDeposit_Pos,intake2TransitionDeposit_Pos);
                break;

            case DEPOSIT:
                setFourbar(intake1Deposit_Pos,intake2Deposit_Pos);
                break;
            case JUNCTION:
                setFourbar(intake1Junction_Pos,intake2Junction_Pos);
        }
    }


    public void setFourbar(double pos1,double pos2)
    {
        intake1.setPosition(pos1);
        intake2.setPosition(pos2);
    }


    public void setFourbarFactor(double factor) {
        double intakeAddition = -0.0008 * factor;
        double intakePos = getAvgIntakePosition();
        //if (!(intakePos + intakeAddition >fourbar_intake_pos ) || !(intakePos - intakeAddition < fourbar_deposit_pos)) {
            intake1.setPosition(intakePos + intakeAddition);
            intake2.setPosition(intakePos + intakeAddition);

    }
    public double getClawPosition()
    {
        return claw.getPosition();
    }
    public double getIntake1Position()
    {
        return intake1.getPosition();
    }
    public double getIntake2Position()
    {
        return intake2.getPosition();
    }
    public double getAvgIntakePosition()
    {
        return (getIntake1Position()+getIntake2Position())/2;
    }
}