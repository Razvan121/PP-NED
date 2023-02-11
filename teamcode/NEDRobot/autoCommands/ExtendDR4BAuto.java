package org.firstinspires.ftc.teamcode.NEDRobot.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.Dr4bGeneralCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class ExtendDR4BAuto extends SequentialCommandGroup {
    public ExtendDR4BAuto (BaseRobot robot, int position){
        super(
                new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                new WaitCommand(300),
                new Dr4bGeneralCommand(robot.dr4bSubsystem,position,10000,10000,10, 1000, Dr4bSubsystem.STATE.FAILED_EXTEND),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.JUNCTION)),
                new WaitCommand(250),
                new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN))
        );
    }
}
