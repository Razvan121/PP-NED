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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.AutoDepositCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.AutoIntakePos;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.ExtendDR4BCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "OpModeRegional", group = "Regionala")
public class CommandRegional extends CommandOpMode {

    private double loopTime = 0;
    private BaseRobot robot;

    private  int HighJunctionPos =  1450;
    private  int MidJunctionPos = 1000;//1000;
    private  int LowJunctionPos = 450;//1000;
    private int GroundJunctionPos = 200;
    private int HomePos = 100;
    private InstantCommand closeClawCommand;
    private InstantCommand openClawCommand;
    private InstantCommand FourBarIntakeCommand;
    private InstantCommand FourBarDepositCommand;
    private InstantCommand FourBarTransitionIntakeCommand;
    private InstantCommand FourBarJunctionCommand;
    private InstantCommand FourBarTransitionDepositCommand;

    public boolean Scoring;
    public boolean InJunction = false;
    public boolean junc=false;


    private ElapsedTime timer;
    private GamepadEx GamepadEx1;
    private GamepadEx GamepadEx2;
    private InstantCommand OdoUpCommand;
    private InstantCommand OdoDownCommand;

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

        FourBarIntakeCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE));

        FourBarDepositCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT));

        FourBarTransitionIntakeCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE));

        FourBarJunctionCommand = new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.JUNCTION));

        FourBarTransitionDepositCommand  = new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT));

        openClawCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN));

        closeClawCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE));

        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

    }
    @Override
    public void run() {

        robot.read();



        //////////////////////////////GAMEPAD1//////////////////////////////////////////////////////////

        if(Scoring) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * 0.4,
                            dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * 0.4,
                            -GamepadEx1.getRightX()* 0.4
                    )
            );
        }
        else
        {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.5 : 0.8),
                            dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.5 : 0.8),
                            -GamepadEx1.getRightX()
                    )
            );
        }


        boolean resetEncoder = gamepad1.dpad_down;

        if(resetEncoder)
        {
            robot.dr4bSubsystem.dr4b_motor.resetEncoder();
        }

        if(gamepad1.left_bumper)
        {
            CommandScheduler.getInstance().schedule(new AutoIntakePos(robot));
        }
        if(gamepad1.right_bumper)
        {
            CommandScheduler.getInstance().schedule(new AutoDepositCommand(robot));
            Scoring=true;

        }
        if(gamepad1.left_trigger>0.5)
        {
            schedule(FourBarJunctionCommand);
        }

        //////////////////////////////GAMEPAD2//////////////////////////////////////////////////////////

        boolean d2DU = gamepad2.dpad_up;
        boolean d2DD = gamepad2.dpad_down;
        boolean d2DL = gamepad2.dpad_left;
        boolean d2DR = gamepad2.dpad_right;

        if(d2DU && !d2DR && !d2DL)
            schedule(FourBarDepositCommand);
        if(d2DD && !d2DR && !d2DL)
            schedule(FourBarIntakeCommand);
        if(d2DL && !d2DU && !d2DD)
            schedule(FourBarTransitionIntakeCommand);
        if(d2DR && !d2DU && !d2DD)
            schedule(FourBarTransitionDepositCommand);


        if(gamepad2.left_bumper)
            schedule(closeClawCommand);


        if(gamepad2.right_bumper) {
            schedule(openClawCommand);
        }

        if(gamepad2.right_trigger>0.5)
        {
            schedule(FourBarJunctionCommand);
        }

        if(gamepad2.left_trigger>0.5)
        {
            schedule(FourBarDepositCommand);
        }

        if(gamepad2.y)
        {
            CommandScheduler.getInstance().schedule(new ExtendDR4BCommand(robot,HighJunctionPos));
            Scoring=true;
        }
        if(gamepad2.x)
        {
            schedule(new ExtendDR4BCommand(robot,MidJunctionPos));
            Scoring=true;
        }

        if(gamepad2.b)
        {
            schedule(new ExtendDR4BCommand(robot, LowJunctionPos));
            Scoring=false;
        }
        if(gamepad2.a)
        {
            schedule(new AutoDepositCommand(robot));
            Scoring = false;
        }
        if(Math.abs(gamepad2.left_stick_y)>0.3)
        {
            robot.dr4bSubsystem.setDr4bFactor(Math.pow(gamepad2.left_stick_y,3));
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
