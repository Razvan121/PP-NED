package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "OpMode")
public class Test extends CommandOpMode {

    private Robot robot;
    private SampleMecanumDrive sampleMecanumDrive;
    private GamepadEx GamepadEx1, GamepadEx2;





    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


        robot = new Robot(hardwareMap,false);
        sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);

        robot.reset();


    }


    @Override
    public void run() {
         super.run();


         sampleMecanumDrive.setWeightedDrivePower(
                 new Pose2d( dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad2.right_trigger > 0.5 ? 0.5 : 1),
                         dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad2.right_trigger > 0.5 ? 0.5 : 1),
                         GamepadEx1.getRightX()
                 )
         );

        boolean upOdo = gamepad1.y;
        boolean downOdo = gamepad1.a;

        if(upOdo)
        {
            schedule(new InstantCommand(() -> robot.odometrySubsystem.UpOdometry()));
        }
        else if(downOdo)
        {
            schedule(new InstantCommand(()->robot.odometrySubsystem.DownOdometry()));
        }

    }


    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
