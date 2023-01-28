package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.AutoDepositCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.ExtendDR4BCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.RetractDR4BCommand;

@Config
@TeleOp
public class CommandRegional extends CommandOpMode {

    private double loopTime = 0;
    private BaseRobot robot;

    private int HighJunctionPos =  1800;
    private static int MidJunctionPos = 0;//1000;
    private int LowJunctionPos = 450;
    private int GroundJunctionPos = 200;
    private int HomePos = 60;

    private InstantCommand closeClawCommand;
    private InstantCommand openClawCommand;
    private InstantCommand FourBarIntakeCommand;
    private InstantCommand FourBarDepositCommand;
    private InstantCommand FourBarTransitionCommand;


    private InstantCommand OdoUpCommand;
    private InstantCommand OdoDownCommand;

    double y,x,rx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new BaseRobot(hardwareMap,false);
        robot.reset();


        OdoDownCommand = new InstantCommand(() ->{
            //robot.odometrySubsystem.update(OdometrySubsystem.OdoState.DOWN);
        });
        OdoUpCommand = new InstantCommand(() ->{
            // robot.odometrySubsystem.update(OdometrySubsystem.OdoState.UP);
        });

        FourBarIntakeCommand = new InstantCommand(() ->{
            robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE);
        });

        FourBarDepositCommand = new InstantCommand(() ->{
            robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT);
        });

        FourBarTransitionCommand = new InstantCommand(() ->{
            robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION);
        });

        openClawCommand = new InstantCommand(() ->{
            robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN);
        });

        closeClawCommand = new InstantCommand(() ->{
            robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE);
        });

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }
    @Override
    public void run() {


        GamepadEx GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx GamepadEx2 = new GamepadEx(gamepad2);

        robot.read();
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

        if (Math.abs(gamepad2.right_stick_y)>0.1) {
            robot.intakeSubsystem.setFourbarFactor((int) Math.pow(-gamepad2.right_stick_y,3));
        }

        if (Math.abs(gamepad2.left_stick_y)>0.1) {
            robot.dr4bSubsystem.setDr4bFactor((int) Math.pow(-gamepad2.left_stick_y,3));
        }


        if(gamepad2.dpad_up&& !gamepad2.dpad_left && !gamepad2.dpad_right)
        {
            schedule(FourBarDepositCommand);
        }

        if(gamepad2.dpad_left && !gamepad2.dpad_down && !gamepad2.dpad_up)
        {
            schedule(FourBarTransitionCommand);
        }

        if(gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right)
        {
            schedule(FourBarIntakeCommand);
        }

        if(gamepad1.left_bumper)
        {
            schedule(closeClawCommand);
        }
        if(gamepad1.right_bumper)
        {
            schedule(new AutoDepositCommand(robot));
        }

        if(gamepad2.y)
        {
            schedule(new ExtendDR4BCommand(robot,HighJunctionPos));
        }
        if(gamepad2.x)
        {
            schedule(new ExtendDR4BCommand(robot,MidJunctionPos));
        }
        if(gamepad2.b)
        {
            schedule(new ExtendDR4BCommand(robot,LowJunctionPos));
        }
        if(gamepad2.a)
        {
            schedule(new RetractDR4BCommand(robot,HomePos));
        }
        robot.dr4bSubsystem.loop();

        CommandScheduler.getInstance().run();

        robot.write();

        telemetry.addData("dr4b_ticks",robot.dr4bSubsystem.getDr4bPosition());
        telemetry.addData("dr4b_velocity",robot.dr4bSubsystem.dr4b_motor.getVelocity());
        telemetry.addData("dr4b_target_pos",robot.dr4bSubsystem.getLiftTargetPosition());
        telemetry.addData("dr4b_power",robot.dr4bSubsystem.dr4b_motor.get());
        telemetry.addData("intake1_pos",robot.intakeSubsystem.getIntake1Position());
        telemetry.addData("intake2_pos",robot.intakeSubsystem.getIntake2Position());
        telemetry.addData("intakeAVG_pos",robot.intakeSubsystem.getAvgIntakePosition());
        telemetry.addData("claw_pos",robot.intakeSubsystem.getClawPosition());
        telemetry.addData("dr4b_state", robot.dr4bSubsystem.liftstate);

        double loop = System.nanoTime();

        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        robot.dr4bSubsystem.dr4b_motor.resetEncoder();
        robot.intakeSubsystem.intake1.setPosition(0);
        robot.intakeSubsystem.intake2.setPosition(0);
        robot.intakeSubsystem.claw.setPosition(0);
    }


    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
