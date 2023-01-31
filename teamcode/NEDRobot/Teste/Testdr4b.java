package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.autoCommands.TeleOPDR4BCommand;

@Config
@TeleOp(name="dr4b")
@Disabled
public class Testdr4b extends CommandOpMode {

    public static int position=0;
    BaseRobot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new BaseRobot(hardwareMap,false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run()
    {
        robot.dr4bSubsystem.read();
        if(gamepad1.dpad_up)
            robot.dr4bSubsystem.setDr4bFactor(1);
        else
            if(gamepad1.dpad_down)
                robot.dr4bSubsystem.setDr4bFactor(-1);



        if(gamepad1.a)
         schedule(new TeleOPDR4BCommand(robot,position, Dr4bSubsystem.STATE.GOOD));

        robot.dr4bSubsystem.loop();
        CommandScheduler.getInstance().run();


        robot.dr4bSubsystem.write();

        telemetry.addData("dr4b_ticks",robot.dr4bSubsystem.getDr4bPosition());
        telemetry.addData("dr4b_ticks_target",robot.dr4bSubsystem.getLiftTargetPosition());
        telemetry.update();
    }

    public void reset()
    {
        CommandScheduler.getInstance().reset();
        robot.dr4bSubsystem.dr4b_motor.resetEncoder();
    }
}
