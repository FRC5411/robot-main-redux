package frc.robot.util;

public class PIDConstants {
    public final double kp;
    public final double ki;
    public final double kd;

    public PIDConstants(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }   

    public double getP(){
        return kp;
    }

    public double getI(){
        return ki;
    }

    public double getD(){
        return kd;
    }
}
