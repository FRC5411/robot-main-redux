package frc.robot.util;

public class FeedForwardConstants {
    public final double ks;
    public final double kv;
    public final double ka;
    public final double kg;

    public FeedForwardConstants(double ks, double kv, double ka, double kg){
        this.ka = ka;
        this.ks = ks;
        this.kv = kv;
        this.kg = kg;
    }   

    public double getA(){
        return ka;
    }

    public double getS(){
        return ks;
    }

    public double getV(){
        return kv;
    }

    public double getG(){
        return kg;
    }
}
