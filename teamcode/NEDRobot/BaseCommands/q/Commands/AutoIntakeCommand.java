package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends SequentialCommandGroup {
    public AutoIntakeCommand(BaseRobot robot){
        super(
                new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                new ExtendDR4BCommand(robot,400)
        );
    }
}
