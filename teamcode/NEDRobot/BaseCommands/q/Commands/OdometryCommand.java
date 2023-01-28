package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.OdometrySubsystem;

public class OdometryCommand extends SequentialCommandGroup {
    public OdometryCommand(BaseRobot robot){
        super(
                new InstantCommand(()-> robot.odometrySubsystem.update(OdometrySubsystem.OdoState.UP))
        );
    }
}
