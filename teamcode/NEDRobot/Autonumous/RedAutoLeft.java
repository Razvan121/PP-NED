package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NEDRobot.Commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedAutoLeft", group="RED")
public class RedAutoLeft extends OpMode {

    private SampleMecanumDrive sampleMecanumDrive;
    private FtcDashboard ftcDashboard;

    //TrajectorySequence
    private TrajectorySequence preload_drop ,pick1,drop1,pick2,drop2,pick3,drop3,pick4,drop4,pick5,drop5,park;

    //Pose
    public static Pose2d POSE_START = new Pose2d(0,0,toRadians(0));

    public static Pose2d[] CYCLE_DROP = new Pose2d[]{
        new Pose2d(0,0,toRadians(0)),//preload
            new Pose2d(0,0,toRadians(0)),//pick1
            new Pose2d(0,0,toRadians(0)),//pick2
            new Pose2d(0,0,toRadians(0)),//pick3
            new Pose2d(0,0,toRadians(0)),//pick4
            new Pose2d(0,0,toRadians(0)),//pick5

    };

    public static Pose2d[] CYCLE_PICK = new Pose2d[]{
            new Pose2d(0,0,toRadians(0)),//drop1
            new Pose2d(0,0,toRadians(0)),//drop2
            new Pose2d(0,0,toRadians(0)),//drop3
            new Pose2d(0,0,toRadians(0)),//drop4
            new Pose2d(0,0,toRadians(0)),//drop5
    };

    public static Pose2d PARK = new Pose2d(0,0,toRadians(0));



    @Override
    public void init() {

        sampleMecanumDrive =  new SampleMecanumDrive(hardwareMap);
        sampleMecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sampleMecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ftcDashboard = FtcDashboard.getInstance();


        //localizer
        sampleMecanumDrive.getLocalizer().setPoseEstimate(POSE_START);

        //trajectory

        //preload_drop = sampleMecanumDrive.trajectorySequenceBuilder(POSE_START)

        //trajectory pick up

        //pick1 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[0])
        //pick2 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[1])
        //pick3 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[2])
        //pick4 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[3])
        //pick5 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[4])

        //trajectory drop

       // drop1 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[0])
        //drop2 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[1])
       // drop3 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[2])
       // drop4 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[3])
       // drop5 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[4])





    }
    @Override
    public void start()
    {
        telemetry.update();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowTrajectoryCommand(sampleMecanumDrive,preload_drop)
                            .alongWith(),


                        new FollowTrajectoryCommand(sampleMecanumDrive,pick1)
                            .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,drop1)
                            .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,pick2)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,drop2)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,pick3)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,drop3)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,pick4)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,drop4)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,pick5)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,drop5)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,park)
                )

        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        sampleMecanumDrive.update();
    }

    @Override
    public void stop()
    {
        CommandScheduler.getInstance().reset();
    }
}
