package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous.ParcareAuto;

import static org.openftc.easyopencv.OpenCvCameraFactory.getInstance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.Timing;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class ParcareAuto extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //public BaseRobot robot1;
    Timing.Timer scoreTimer;


    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

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

    int LEFT = 6 ; // Tag ID 18 from the 36h11 family
    int MIDDLE = 7;
    int RIGHT= 8;

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        telemetry.setMsTransmissionInterval(50);

        robot.init(hardwareMap);
       // robot1 = new BaseRobot(hardwareMap,true);

        Pose2d startPose = new Pose2d(0, 0, 0);
        if(isStopRequested())
            return;

        while (!isStarted() && !isStopRequested())
        {

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

            telemetry.update();
            sleep(20);
        }
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            robot.setAllPower(0.5);
            scoreTimer = new Timing.Timer(2300);
            scoreTimer.start();
            while(!scoreTimer.done()) {

            }
            scoreTimer.pause();
            robot.setAllPower(0);
            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            robot.setAllPower(-0.5);
            scoreTimer = new Timing.Timer(1100);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            robot.setAllPower(0);
            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();

            turnPID(90);
            scoreTimer = new Timing.Timer(700);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            robot.setAllPower(0.5);
            scoreTimer = new Timing.Timer(1500);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();
            robot.setAllPower(0);
            scoreTimer = new Timing.Timer(200);
            scoreTimer.start();
            while (!scoreTimer.done()) {

            }
            scoreTimer.pause();



        }


        }else if(tagOfInterest.id == MIDDLE){
        robot.setAllPower(0.5);
        scoreTimer = new Timing.Timer(2300);
        scoreTimer.start();
        while(!scoreTimer.done()) {

        }
        scoreTimer.pause();
        robot.setAllPower(0);
        scoreTimer = new Timing.Timer(200);
        scoreTimer.start();
        while (!scoreTimer.done()) {

        }
        scoreTimer.pause();
        robot.setAllPower(-0.5);
        scoreTimer = new Timing.Timer(1100);
        scoreTimer.start();
        while (!scoreTimer.done()) {

        }
        scoreTimer.pause();
        robot.setAllPower(0);
        scoreTimer = new Timing.Timer(200);
        scoreTimer.start();
        while (!scoreTimer.done()) {

        }
        scoreTimer.pause();
        }else
        {
            robot.setAllPower(0.5);
        scoreTimer = new Timing.Timer(2300);
        scoreTimer.start();
        while(!scoreTimer.done()) {

        }
        scoreTimer.pause();
        robot.setAllPower(0);
        scoreTimer = new Timing.Timer(200);
        scoreTimer.start();
        while (!scoreTimer.done()) {

        }
        scoreTimer.pause();
        robot.setAllPower(-0.5);
        scoreTimer = new Timing.Timer(1100);
        scoreTimer.start();
        while (!scoreTimer.done()) {

        }
        scoreTimer.pause();
        robot.setAllPower(0);
        scoreTimer = new Timing.Timer(200);
        scoreTimer.start();
        while (!scoreTimer.done()) {

        }
        scoreTimer.pause();
            turnPID(-90);
            scoreTimer = new Timing.Timer(700);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            robot.setAllPower(0.4);
            scoreTimer = new Timing.Timer(1000);
            scoreTimer.start();
            while (!scoreTimer.done())
            {

            }
            scoreTimer.pause();
            robot.setAllPower(0);
            turnPID(90);
        }

*/

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
    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }



    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 4 || pid.getLastSlope() > 1) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }

   /* public void setPower(double p)
    {
        robot1.drive.setMotorPowers(p,p,p,p);
    }

    */

}