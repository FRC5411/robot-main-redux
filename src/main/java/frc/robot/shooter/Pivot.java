package frc.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private TalonFX rightPivotMotor;
    private TalonFX leftPivotMotor;
    private DutyCycleEncoder pivotEncoder;
    private MotionMagic

    public Pivot(){
        rightPivotMotor = new TalonFX(ShooterConstants.pivotConstants.kRightPivotPort);
        leftPivotMotor = new TalonFX(ShooterConstants.pivotConstants.kLeftPivotPort);
        pivotEncoder = new DutyCycleEncoder(ShooterConstants.pivotConstants.kAbsoulteEncoderPort);

        config();
    }

    private void config(){
        // Factory reset the krakens 
        var reset = new TalonFXConfiguration();

        rightPivotMotor.getConfigurator().apply(reset);
        leftPivotMotor.getConfigurator().apply(reset);

        rightPivotMotor.getConfigurator().apply(ShooterConstants.configs.rightConfig);
        leftPivotMotor.getConfigurator().apply(ShooterConstants.configs.leftConfig);
    }

    public void setPivotPosistion(double demand){
        rightPivotMotor.setControl(new PositionDutyCycle(demand));
        // This allows us to make the leftPivotMotor do whatever the right pivot motor does
        // the pivot motors were already set up to follow the same direction no need to flip
        leftPivotMotor.setControl(new Follower(ShooterConstants.pivotConstants.kLeftPivotPort, false));
    } 

    public double getPivotPosistion(){
        return 0;
    }

    @Override
    public void periodic(){

    }
}
