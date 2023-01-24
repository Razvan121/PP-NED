package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LiftSubsystem extends SubsystemBase {

    public final MotorEx lift;

    private MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;


    private double voltage;
    private int liftPosition;

    private final double P=0;
    private final double I=0;
    private final double D=0;
    public int endpos=0;

    public double power = 0.0;
    public double targetPosition = 0.0;
    public int getTargetPos()
    {
        return endpos;
    }

    public enum STATE{
        READY,
        LOW,
        MEDIUM,
        HIGH,
        RETRACT;
    }




    public LiftSubsystem(HardwareMap hardwareMap,boolean isAuto) {
        this.lift = new MotorEx(hardwareMap,"lift");

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.controller = new PIDController(P,I,D);
        controller.setPID(P,I,D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.timer = new ElapsedTime();
        timer.reset();
    }

    public void loop() {
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
        }

        power = -controller.calculate(liftPosition, targetPosition) / voltage * 14;
    }

    public void write()
    {
        lift.set(power);
    }

    public int getPos() {
        return liftPosition;
    }

    public void setSlideFactor(double factor) {
        double slideAddition = 20 * factor;
        double newPosition = liftPosition + slideAddition;
        if (curState.getV() == 0 && newPosition >= 0 && newPosition <= 603) {
            targetPosition = newPosition;
        }
    }

    public void resetTimer() {
        timer.reset();
    }


    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(targetPos, 0), max_v, max_a);
         endpos = (int) targetPos;
        resetTimer();
    }
}