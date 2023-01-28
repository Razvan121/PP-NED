package org.firstinspires.ftc.teamcode.NEDRobot.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class TeleOPIntakeCommand extends SequentialCommandGroup {
    public TeleOPIntakeCommand(BaseRobot robot){
        super(
                new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(300),
                new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE))
        );
    }
}
