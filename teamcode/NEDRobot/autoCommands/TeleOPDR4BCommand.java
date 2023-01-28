package org.firstinspires.ftc.teamcode.NEDRobot.autoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.Dr4bGeneralCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;

public class TeleOPDR4BCommand extends ParallelCommandGroup {
    public TeleOPDR4BCommand (BaseRobot robot, int position, Dr4bSubsystem.STATE state){
        super(
                new Dr4bGeneralCommand(robot.dr4bSubsystem,position,10000,10000,500, 10000,state)


        );
    }
}
