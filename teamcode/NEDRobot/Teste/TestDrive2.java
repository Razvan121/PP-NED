package org.firstinspires.ftc.teamcode.NEDRobot.Teste;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.autoCommands.TeleOPDR4BCommand;

@TeleOp(name = "TestDrive2")
@Disabled
public class TestDrive2 extends CommandOpMode {
    //private SampleMecanumDrive drive;
    private BaseRobot robot;
    private GamepadEx GamepadEx1, GamepadEx2;


    private static int position = 1450;





    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

       // drive = new SampleMecanumDrive(hardwareMap);
        robot =  new BaseRobot(hardwareMap,false);
        robot.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        GamepadEx   GamepadEx1;
        GamepadEx  GamepadEx2;

    }


    @Override
    public void run() {

        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);

        robot.write();

         super.run();


         robot.drive.setWeightedDrivePower(
                 new Pose2d( dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.5 : 1),
                         dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.5 : 1),
                         -GamepadEx1.getRightX()
                 )
         );
         boolean d1DD = gamepad1.dpad_down;

         if(d1DD)
         {
             robot.dr4bSubsystem.dr4b_motor.resetEncoder();
         }

       // boolean upOdo = gamepad1.y;
      //  boolean downOdo = gamepad1.a;

        /* if(upOdo)
        {
            schedule(new InstantCommand(() -> drive.odometrySubsystem.update(OdometrySubsystem.OdoState.UP)));
        }
        else if(downOdo)
        {
            schedule(new InstantCommand(()->robot.odometrySubsystem.update(OdometrySubsystem.OdoState.DOWN)));
        }

         */

        boolean d2DU = gamepad2.dpad_up;
        boolean d2DD = gamepad2.dpad_down;

        if(d2DU)
        {
            robot.dr4bSubsystem.setDr4bFactor(1);
        }
        else
            if(d2DD)
            {
                robot.dr4bSubsystem.setDr4bFactor(-1);
            }


        boolean d2Y = gamepad2.y;
        boolean d2A = gamepad2.a;

        if(d2Y)
        {
            schedule(new TeleOPDR4BCommand(robot,position, Dr4bSubsystem.STATE.GOOD));
        }
        if(d2A)
        {
            schedule(new TeleOPDR4BCommand(robot,0,Dr4bSubsystem.STATE.GOOD));
        }


        robot.dr4bSubsystem.loop();

        CommandScheduler.getInstance().run();

        robot.write();


        //telemetry
        telemetry.addData("Dr4bCurrPos",robot.dr4bSubsystem.dr4b_motor.motorEx.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Dr4bPos",robot.dr4bSubsystem.getDr4bPosition());
        telemetry.addData("Dr4bTarget",robot.dr4bSubsystem.getLiftTargetPosition());
        telemetry.addData("Dr4bPower",robot.dr4bSubsystem.power);
        telemetry.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    public void reset()
    {
        CommandScheduler.getInstance().reset();
        robot.dr4bSubsystem.dr4b_motor.resetEncoder();
        robot.dr4bSubsystem.dr4b_motor.set(0);

    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
