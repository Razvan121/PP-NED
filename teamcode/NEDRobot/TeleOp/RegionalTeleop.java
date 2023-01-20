package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Robot;

import java.text.DecimalFormat;

@TeleOp(name = "OpMode")
public class RegionalTeleop extends CommandOpMode {

    private Robot robot;
    private double drive=0,turn=0, strafe=0,speed=0.6;
    private boolean last_left_bumper = false;
    private boolean last_right_bumper = false;
    private double powerFR, powerFL, powerRR, powerRL;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


       robot = new Robot(hardwareMap,false);
       robot.reset();

        robot.motorFrontRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorBackLeft.setPower(0);
    }


    @Override
    public void run() {
        takeControllerInput();
        drive();


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

    private void takeControllerInput() {
        drive = -1 * gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        turn = gamepad1.left_stick_x;
        if (gamepad1.left_bumper && last_left_bumper != gamepad1.left_bumper)
            speed = Math.max(0.15, speed - 0.15);
        if (gamepad1.right_bumper && last_right_bumper != gamepad1.right_bumper)
            speed = Math.min(1, speed + 0.15);

        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper =gamepad1.right_bumper;
    }


    private void addTurn(double turn) {
        powerFR -= turn;
        powerRR -= turn;
        powerFL += turn;
        powerRL += turn;
    }


    private void drive() {

        powerFR = drive - strafe;
        powerFL = drive + strafe;
        powerRR = drive + strafe;
        powerRL = drive - strafe;

        addTurn(turn);

        // multiplies by speed
        powerFR *= speed;
        powerFL *= speed;
        powerRR *= speed;
        powerRL *= speed;

        // applies the power
        robot.motorFrontRight.setPower(powerFR);
        robot.motorFrontLeft.setPower(powerFL);
        robot.motorBackRight.setPower(powerRR);
        robot.motorBackLeft.setPower(powerRL);


        DecimalFormat df = new DecimalFormat("#%");
        telemetry.addData("speed", df.format(speed));
        telemetry.update();
    }

}
