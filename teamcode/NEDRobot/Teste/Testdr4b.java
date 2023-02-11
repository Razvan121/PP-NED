package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="dr4b")
public class Testdr4b extends OpMode {

    public static double P=0.000;
    public static double I=0.0;
    public static double D=0.000;
    public static double F=0.00025;


    public Servo intake1,intake2;

    public MotionProfile profile;
    public MotionState currentState;

    private ElapsedTime timer;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private PIDFController controller;

    MotorEx dr4b_motor;

    private int dr4bPosition=0;
    public static double targetPosition=0;
    public double power=0;
    private double voltage;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dr4b_motor = new MotorEx(hardwareMap,"dr4b");
        intake1 =  hardwareMap.get(Servo.class,"intake1");
        intake2 =  hardwareMap.get(Servo.class,"intake2");

        intake1.setDirection(Servo.Direction.REVERSE);

        timer = new ElapsedTime();
        timer.reset();

        voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1,0),new MotionState(0,0),30,25);

        controller = new PIDFController(P,I,D,F);
        controller.setPIDF(P,I,D,F);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
        dr4b_motor.resetEncoder();
    }
    @Override
    public void loop()
    {
        dr4bPosition=dr4b_motor.getCurrentPosition();
        if(gamepad1.a)
        {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(dr4bPosition,0),new MotionState(targetPosition,0),3000,3000);
            timer.reset();
        }

        controller = new PIDFController(P,I,D,F);
        controller.setPIDF(P,I,D,F);

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        currentState=profile.get(timer.time());
        if(currentState.getV() != 0)
        {
            targetPosition= currentState.getX();
        }

        power = controller.calculate(dr4bPosition,targetPosition)/ voltage*12 ;

        dr4b_motor.set(power);


        telemetry.addData("dr4b_ticks",dr4bPosition);
        telemetry.addData("dr4b_ticks_target",targetPosition);
        telemetry.addData("time",timer.milliseconds());
        telemetry.addData("vel",dr4b_motor.getVelocity());
        telemetry.addData("power",power);

        telemetry.update();
    }

    public void reset()
    {
        dr4b_motor.resetEncoder();
    }
}
