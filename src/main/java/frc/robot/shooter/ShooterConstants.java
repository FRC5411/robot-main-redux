package frc.robot.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.FeedForwardConstants;
import frc.robot.util.PIDConstants;
import frc.robot.util.ShooterPosistion;

public class ShooterConstants{

    public class pivotConstants{
        public static final int kPivotPort = 0;
        public static final double kPivotGearRatio = 1.0/1.0;

        public static final int kAbsoulteEncoderPort = 0;

        public static final PIDConstants pivotControllerConstants = new PIDConstants(0, 0, 0);
        public static final FeedForwardConstants pivotControllerFFConstants = new FeedForwardConstants(0, 0, 0, 0);
    }

    public class flywheelConstants{
        public static final int kTRWheelPort = 0;
        public static final int kBRWheelPort = 0;
        public static final int kTLWheelPort = 0;
        public static final int kBLWheelPort = 0;

        public static final double kWheelGearRatio = 1.0/1.0;

        public static final PIDConstants flyWheelControllerConstants = new PIDConstants(0, 0, 0);

    }

    public static final ShooterPosistion subwooferPosistion = new ShooterPosistion(new Rotation2d(0), 0);

    
}