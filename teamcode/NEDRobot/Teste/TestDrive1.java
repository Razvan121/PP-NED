package org.firstinspires.ftc.teamcode.NEDRobot.Teste;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.SampleMecanumDrive;

import java.text.DecimalFormat;

@TeleOp(name = "TestDrive1")
public class TestDrive1 extends LinearOpMode {

    private SampleMecanumDrive drive1;
    private double drive = 0, turn = 0, strafe = 0, speed = 0.6;
    private boolean last_left_bumper = false;
    private boolean last_right_bumper = false;
    private double powerFR, powerFL, powerRR, powerRL;

    @Override
    public void runOpMode() throws InterruptedException {
       // CommandScheduler.getInstance().reset();
         drive1 = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            takeControllerInput();
            drive();
            boolean upOdo = gamepad1.y;
            boolean downOdo = gamepad1.a;

          /*  if (upOdo)
                robot.odometrySubsystem.update(OdometrySubsystem.OdoState.UP);
            if (downOdo)
                robot.odometrySubsystem.update(OdometrySubsystem.OdoState.DOWN);

           */
        }
    }


    private void takeControllerInput() {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        if (gamepad1.left_bumper && last_left_bumper != gamepad1.left_bumper)
            speed = Math.max(0.15, speed - 0.15);
        if (gamepad1.right_bumper && last_right_bumper != gamepad1.right_bumper)
            speed = Math.min(1, speed + 0.15);

        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;
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
        drive1.setMotorPowers(powerFL, powerRL, powerRR, powerFR);

        DecimalFormat df = new DecimalFormat("#%");
        telemetry.addData("speed", df.format(speed));
        telemetry.update();
    }
}
