    package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

    import com.acmerobotics.dashboard.config.Config;
    import com.arcrobotics.ftclib.command.SubsystemBase;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.Servo;

    @Config
    public class OdometrySubsystem extends SubsystemBase {

        private Servo LeftOdo,RightOdo,FrontOdo;
        public enum OdoState {
            UP,
            DOWN
        }
        private double odo_up_pos1=0.14,odo_up_pos2=0.305,odo_up_pos3=1;
        private double odo_down_pos1=0.23,odo_down_pos2=0.23,odo_down_pos3=0.6;

        public OdometrySubsystem(HardwareMap hardwareMap,boolean isAuto){
            this.LeftOdo = hardwareMap.get(Servo.class,"LeftOdo");
            this.RightOdo = hardwareMap.get(Servo.class,"RightOdo");
            this.FrontOdo = hardwareMap.get(Servo.class,"FrontOdo");
            this.FrontOdo.setDirection(Servo.Direction.REVERSE);
            this.RightOdo.setDirection(Servo.Direction.REVERSE);

            if(isAuto)
            {
                update(OdoState.UP);
                update(OdoState.DOWN);
            }
            else
            {
                update(OdoState.UP);
            }
        }

        public void update(OdoState state)
        {
            switch (state){
                case UP:
                    setOdo(odo_up_pos1,odo_up_pos2,odo_up_pos3);
                    break;
                case DOWN:
                    setOdo(odo_down_pos1,odo_down_pos2,odo_down_pos3);
                    break;
            }
        }
        public void setOdo(double pos1,double pos2, double pos3)
        {
            FrontOdo.setPosition(pos1);
            LeftOdo.setPosition(pos2);
            RightOdo.setPosition(pos3);
        }
    }
