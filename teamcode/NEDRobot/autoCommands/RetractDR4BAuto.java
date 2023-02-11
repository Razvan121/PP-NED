package org.firstinspires.ftc.teamcode.NEDRobot.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.Dr4bGeneralCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class RetractDR4BAuto extends SequentialCommandGroup {
    public RetractDR4BAuto(BaseRobot robot, double pos ){
        super(
                new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE)),
                new Dr4bGeneralCommand(robot.dr4bSubsystem,0,10000,10000,10,2000, Dr4bSubsystem.STATE.FAILED_RETRACT),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intakeSubsystem.setFourbar(pos,pos))

        );
    }
}
