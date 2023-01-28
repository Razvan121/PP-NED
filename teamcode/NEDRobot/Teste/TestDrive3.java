package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
@TeleOp
public class TestDrive3 extends CommandOpMode {
    private BaseRobot robot;
   // private SampleMecanumDrive drive1;
    double y,x,rx;
    private double loopTime = 0;

    @Override
    public void initialize()
    {
        robot = new BaseRobot(hardwareMap,false);
       // drive1 = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }
    public void run()
    {
        if(Math.abs(gamepad1.left_stick_y)>0.02)
            y = -gamepad1.left_stick_y;
        else
            y = 0;
        if(Math.abs(gamepad1.left_stick_x)>0.02)
            x = gamepad1.left_stick_x * 1.1;
        else
            x=0;
        if(Math.abs(gamepad1.right_stick_x)>0.02)
            rx = gamepad1.right_stick_x/1.1;
        else
            rx=0;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double speedFrontLeft = ((y + x + rx) / denominator);
        double speedBackLeft = ((y - x + rx) / denominator);
        double speedFrontRight = ((y - x - rx)/denominator);
        double speedBackRight = ((y + x - rx)/denominator);

        robot.drive.setMotorPowers(speedFrontLeft,speedBackLeft,speedBackRight,speedFrontRight);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}
