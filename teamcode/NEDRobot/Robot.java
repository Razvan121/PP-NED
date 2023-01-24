package org.firstinspires.ftc.teamcode.NEDRobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.OdometrySubsystem;

public class Robot {
    private final Object imuLock = new Object();
    public final BNO055IMU imu;
    private Thread imuThread;
    private VoltageSensor voltageSensor;


    public DcMotor motorFrontLeft,motorBackLeft,motorFrontRight,motorBackRight;
    public OdometrySubsystem odometrySubsystem;
    public IntakeSubsystem intakeSubsystem;

    private boolean isAuto = false;

    public Robot(HardwareMap hardwareMap,boolean isAuto)
    {
        this.isAuto = isAuto;

        motorFrontLeft = hardwareMap.dcMotor.get("FS");
        motorBackLeft = hardwareMap.dcMotor.get("SS");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackRight = hardwareMap.dcMotor.get("SD");

        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap,false);


        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        synchronized (imuLock){
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

    }

    public void reset()
    {

    }


}
