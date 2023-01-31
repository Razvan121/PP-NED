package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class AutoDriver1Command extends SequentialCommandGroup {
    public AutoDriver1Command (BaseRobot robot){
        super(
                new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(500),
                new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE))
        );
    }
}
