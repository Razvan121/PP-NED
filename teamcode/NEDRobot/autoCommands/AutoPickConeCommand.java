package org.firstinspires.ftc.teamcode.NEDRobot.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.AutoTakeFourBarCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;

public class AutoPickConeCommand extends SequentialCommandGroup {
    public AutoPickConeCommand(BaseRobot robot,double positon){
        super(
                new SequentialCommandGroup(
                    new InstantCommand(()->robot.intakeSubsystem.setFourbar(positon)),
                        new WaitCommand(500),
                        new AutoTakeFourBarCommand(robot)
                )
        );
    }
}
