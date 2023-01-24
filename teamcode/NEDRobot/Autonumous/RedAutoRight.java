package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NEDRobot.Commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Robot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "RedAutoRight", group = "RED")
public class RedAutoRight extends LinearOpMode  {

    private SampleMecanumDrive sampleMecanumDrive;
    private Robot robot;
    private FtcDashboard ftcDashboard;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;


    //TrajectorySequence
    private TrajectorySequence preload_drop ,pick1,drop1,pick2,drop2,pick3,drop3,pick4,drop4,pick5,drop5,park1,park2,park3;

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

    public static Pose2d PARK[] = new Pose2d[]{
            new Pose2d(0,0,toRadians(0)),
            new Pose2d(0,0,toRadians(0)),
            new Pose2d(0,0,toRadians(0)),
    };

    public RedAutoRight() {
    }


    @Override
    public void runOpMode() throws InterruptedException {

        sampleMecanumDrive =  new SampleMecanumDrive(hardwareMap);
        sampleMecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sampleMecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot = new Robot(hardwareMap,true);

        // TRAJECTORY

        preload_drop = sampleMecanumDrive.trajectorySequenceBuilder(POSE_START)
                .build();

        //trajectory pick up

        pick1 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[0])
                .build();
        pick2 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[1])
                .build();
        pick3 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[2])
                .build();
        pick4 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[3])
                .build();
        pick5 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_PICK[4])
                .build();

        //trajectory drop

        drop1 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[0])
                 .build();
        drop2 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[1])
                .build();
        drop3 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[2])
                .build();
        drop4 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[3])
                .build();
        drop5 = sampleMecanumDrive.trajectorySequenceBuilder(CYCLE_DROP[4])
                .build();


        park1 = sampleMecanumDrive.trajectorySequenceBuilder(PARK[0])
                .build();
        park2 = sampleMecanumDrive.trajectorySequenceBuilder(PARK[1])
                .build();
        park3 = sampleMecanumDrive.trajectorySequenceBuilder(PARK[2])
                .build();


        sampleMecanumDrive.odometrySubsystem.DownOdometry();
        robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE);

        waitForStart();

        if(isStopRequested())
            return;


        camera.stopStreaming();

        sampleMecanumDrive.odometrySubsystem.DownOdometry();

        //localizer
        sampleMecanumDrive.getLocalizer().setPoseEstimate(POSE_START);

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

                        new FollowTrajectoryCommand(sampleMecanumDrive,park1)

                )

        );

        CommandScheduler.getInstance().run();


    }
}
