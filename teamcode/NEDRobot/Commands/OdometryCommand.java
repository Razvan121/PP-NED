package org.firstinspires.ftc.teamcode.NEDRobot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.OdometrySubsystem;

public class OdometryCommand extends SequentialCommandGroup {
    public OdometryCommand(OdometrySubsystem odometrySubsystem){
        super(
                new InstantCommand(()-> odometrySubsystem.UpOdometry())
        );
    }
}
