package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    public Servo claw;
    public Servo intake1;
    public Servo intake2;


    private double closePos=0.44;
    private double openPos=0.51;

    private double fourbar_intake_pos=0.62;
    private  double fourbar_transition_intake_pos =0.60;
    private double fourbar_transition_deposit_pos = 0.55;
    private double fourbar_deposit_pos=0.52;
    private double fourbar_junction_pos = 0.51;




    public enum ClawState {
        CLOSE,
        OPEN,
    }
    public enum FourbarState{
        INTAKE,
        TRANSITION_INTAKE,
        JUNCTION,
        DEPOSIT,
        TRANSITION_DEPOSIT
    }


    public IntakeSubsystem(HardwareMap hardwareMap,boolean isAuto) {
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.intake1 = hardwareMap.get(Servo.class, "intake1");
        this.intake2 = hardwareMap.get(Servo.class, "intake2");
        this.intake1.setDirection(Servo.Direction.REVERSE);

        if(isAuto)
        {
            update(ClawState.CLOSE);
            update(FourbarState.INTAKE);
        }
        else
        {
            update(ClawState.OPEN);
            update(FourbarState.TRANSITION_DEPOSIT);
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
                setFourbar(fourbar_intake_pos);
                break;
            case TRANSITION_INTAKE:
                setFourbar(fourbar_transition_intake_pos);
                break;
            case TRANSITION_DEPOSIT:
                setFourbar(fourbar_transition_deposit_pos);
                break;
            case JUNCTION:
                setFourbar(fourbar_junction_pos);
                break;
            case DEPOSIT:
                setFourbar(fourbar_deposit_pos);
        }
    }
    public void setFourbar(double pos)
    {
        intake1.setPosition(pos);
        intake2.setPosition(pos+0.007);
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