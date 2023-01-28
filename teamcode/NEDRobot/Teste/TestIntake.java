package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TestIntake extends OpMode {
    public Servo intake1,intake2;
    public static double pos1=0.5,pos2=0.5;
    public double pos2_deposit=0.4,pos2_intake=0.55;
    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake1 =  hardwareMap.get(Servo.class,"intake1");
        intake2 =  hardwareMap.get(Servo.class,"intake2");
        intake1.setDirection(Servo.Direction.REVERSE);
        //intake2.setDirection(Servo.Direction.REVERSE);
    //    pos1=0.455  pos2=0.532


        // pos1 == pos2 => pos2 = 0.532,pos1 = 0.4
        // pos2 = 0.4 , pos1 = 0.601


        //pos1 = 0.43 creste
        //pos2 = 1-0.569 = 0.431

        //pos1 = 0.4
        //pos2 = 1-0.601 = 0.399



    }
    @Override
    public void loop(){



        intake1.setPosition(pos1);
        intake2.setPosition(pos1+0.005  );//1-pos1
        telemetry.addData("Pos1",intake1.getPosition());
        telemetry.addData("Pos2",intake2.getPosition());
        telemetry.update();
    }
}
