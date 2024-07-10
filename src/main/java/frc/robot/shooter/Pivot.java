package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.proto.ArmFeedforwardProto;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private CANSparkMax pivotMotor;
    private SparkPIDController pivotController;
    private DutyCycleEncoder pivotEncoder;
    private ArmFeedforward pivotFF;

    public Pivot(){
        pivotMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);
        pivotEncoder = new DutyCycleEncoder(ShooterConstants.pivotConstants.kAbsoulteEncoderPort);
        config();
    }

    private void config(){
        
    }

    public void setPivotPosistion(double demand){
    } 

    public double getPivotPosistion(){
        return 0;
    }

    @Override
    public void periodic(){

    }
}
