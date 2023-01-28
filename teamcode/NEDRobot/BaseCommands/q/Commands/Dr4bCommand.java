package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.Dr4bGeneralCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;

public class Dr4bCommand extends SequentialCommandGroup {
    public Dr4bCommand(BaseRobot robot, double position, Dr4bSubsystem.STATE state)
    {
        super(
          new Dr4bGeneralCommand(robot.dr4bSubsystem,position,10000,10000,10,1000,state)
        );
    }
}
