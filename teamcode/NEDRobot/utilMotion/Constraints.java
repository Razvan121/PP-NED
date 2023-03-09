package org.firstinspires.ftc.teamcode.NEDRobot.utilMotion;

public class Constraints {
    public double accel;
    public double decel;
    public double velo;

    public Constraints(double velo, double accel) {
        this(accel, accel, velo);
    }

    public Constraints(double velo, double accel, double decel) {
        this.accel = Math.abs(accel);
        this.decel = Math.abs(decel);
        this.velo = Math.abs(velo);
    }
}
