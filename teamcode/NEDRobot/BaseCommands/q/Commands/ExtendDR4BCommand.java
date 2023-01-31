package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.Dr4bGeneralCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class ExtendDR4BCommand extends ParallelCommandGroup {
    public ExtendDR4BCommand (BaseRobot robot, int position){
        super(
                new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                new WaitCommand(300),
                new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                new WaitCommand(500),
                new Dr4bGeneralCommand(robot.dr4bSubsystem,position,3000,2500,10, 1000, Dr4bSubsystem.STATE.FAILED_EXTEND)
        );
    }
}
