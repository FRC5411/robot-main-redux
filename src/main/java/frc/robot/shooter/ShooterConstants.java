package frc.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.FeedForwardConstants;
import frc.robot.util.PIDConstants;
import frc.robot.util.ShooterPosistion;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterConstants{

    public class pivotConstants{
        public static final int kRightPivotPort = 0;
        public static final int kLeftPivotPort = 0;
        public static final double kPivotGearRatio = 1.0/1.0;
        public static final double kPivotPosistionConversionFactor = 1.0/1.0;

        public static final int kAbsoulteEncoderPort = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;

    }

    public class flywheelConstants{
        public static final int kTRWheelPort = 0;
        public static final int kBRWheelPort = 0;
        public static final int kTLWheelPort = 0;
        public static final int kBLWheelPort = 0;

        public static final double kWheelGearRatio = 1.0/1.0;

        public static final PIDConstants flyWheelControllerConstants = new PIDConstants(0, 0, 0);
        public static final FeedForwardConstants flywheelControlllerFFConstants = new FeedForwardConstants(0, 0, 0, 0);

    }

    public class configs{
        public static TalonFXConfiguration leftConfig;
        public static TalonFXConfiguration rightConfig;

        public configs(){
            leftConfig = new TalonFXConfiguration();
            rightConfig = new TalonFXConfiguration();

            leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            leftConfig.CurrentLimits.StatorCurrentLimit = 60;
            leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            leftConfig.CurrentLimits.SupplyCurrentLimit = 60;
            leftConfig.Voltage.PeakForwardVoltage = 12.0;
            leftConfig.Voltage.PeakReverseVoltage = -12.0;
            leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            // TODO : Check this
            leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            leftConfig.Feedback.FeedbackRemoteSensorID = (ShooterConstants.pivotConstants.kAbsoulteEncoderPort);

            leftConfig.Slot0.kP = ShooterConstants.pivotConstants.kP;
            leftConfig.Slot0.kG = ShooterConstants.pivotConstants.kG;
            leftConfig.Slot0.kD = ShooterConstants.pivotConstants.kD;
            leftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            
            rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            rightConfig.CurrentLimits.StatorCurrentLimit = 60;
            rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            rightConfig.CurrentLimits.SupplyCurrentLimit = 60;
            rightConfig.Voltage.PeakForwardVoltage = 12.0;
            rightConfig.Voltage.PeakReverseVoltage = -12.0;
            rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            rightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            rightConfig.Feedback.FeedbackRemoteSensorID = (ShooterConstants.pivotConstants.kAbsoulteEncoderPort);
    
            rightConfig.Slot0.kP = ShooterConstants.pivotConstants.kP;
            rightConfig.Slot0.kG = ShooterConstants.pivotConstants.kG;
            rightConfig.Slot0.kD = ShooterConstants.pivotConstants.kD;
            rightConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        }
    }



    public static final ShooterPosistion subwooferPosistion = new ShooterPosistion(Rotation2d.fromDegrees(1), 0);

    
}