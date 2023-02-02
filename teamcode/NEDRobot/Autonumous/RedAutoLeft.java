package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.AutoDepositCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.ExtendDR4BCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RedAutoLeft", group="RED")
public class RedAutoLeft extends OpMode {

    private SampleMecanumDrive sampleMecanumDrive;
    private FtcDashboard ftcDashboard;
    private BaseRobot robot;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private double fourbarFIRSTCONE= 0.54;
    private  double fourbarSECONDCONE=0.55;
    private double fourbarTHIRDCONE=0.56;
    private double fourbarFOURTHCONE=0.57;
    private double fourbarLASTCONE = 0.58;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1 ; // Tag ID 18 from the 36h11 family
    int MIDDLE = 2;
    int RIGHT= 3;

    AprilTagDetection tagOfInterest = null;

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
        //Robot robot = new Robot(hardwareMap,true);
        sampleMecanumDrive =  new SampleMecanumDrive(hardwareMap);
        sampleMecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sampleMecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot = new BaseRobot(hardwareMap,true);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30);


        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        //sampleMecanumDrive..DownOdometry();

        //localizer
        sampleMecanumDrive.getLocalizer().setPoseEstimate(POSE_START);

        //trajectory

        //preload_drop = sampleMecanumDrive.trajectorySequenceBuilder(POSE_START)

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

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            PARK = new Pose2d(0,0,toRadians(0));
        }else if(tagOfInterest.id == MIDDLE){
            PARK = new Pose2d(0,0,toRadians(0));
        }else{
            PARK = new Pose2d(0,0,toRadians(0));
        }

        telemetry.update();


    }
    @Override
    public void start()
    {



        telemetry.update();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowTrajectoryCommand(sampleMecanumDrive,preload_drop)
                                .alongWith(new ExtendDR4BCommand(robot, 1600)).andThen(new WaitCommand(500))
                                .andThen(new AutoDepositCommand(robot)),


                        new FollowTrajectoryCommand(sampleMecanumDrive,pick1)
                                .alongWith(),

                        new FollowTrajectoryCommand(sampleMecanumDrive,drop1)
                                .alongWith(new ExtendDR4BCommand(robot, 1600)).andThen(new WaitCommand(500))
                                .andThen(new AutoDepositCommand(robot)),

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
        camera.closeCameraDevice();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}