package org.firstinspires.ftc.teamcode.NEDRobot.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Robot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class TeleOPIntakeCommand extends SequentialCommandGroup {
    public TeleOPIntakeCommand(Robot robot){
        super(
                new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(()->robot.intakeSubsystem.setFourbar(IntakeSubsystem.fourbar_extended))
        );
    }
}
