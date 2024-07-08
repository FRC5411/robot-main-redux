package frc.robot.shooter;

import frc.robot.util.PIDConstants;

public class ShooterConstants{

    public class pivotConstants{
        public static final int kPivotPort = 0;
        public static final double kPivotGearRatio = 1.0/1.0;

        public static final int kAbsoulteEncoderPort = 0;
    }

    public class flywheelConstants{
        public static final int kTRWheelPort = 0;
        public static final int kBRWheelPort = 0;
        public static final int kTLWheelPort = 0;
        public static final int kBLWheelPort = 0;

        public static final double kWheelGearRatio = 1.0/1.0;

        public static final PIDConstants flyWheelConstants = new PIDConstants(0, 0, 0);

    }

    
}