// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax pivotMotor;
  // TODO: Find a better name gang
  private CANSparkMax topRightWheelMotor;
  private CANSparkMax topLeftWheelMotor;
  private CANSparkMax bottomRightWheelMotor;
  private CANSparkMax bottomLeftWheelMotor;

  private SparkPIDController pivotController;
  private SparkPIDController topRightController;
  private SparkPIDController topLeftController;
  private SparkPIDController bottomRightController;
  private SparkPIDController bottomLeftController;

  private RelativeEncoder topRightEncoder;
  private RelativeEncoder topLeftEncoder;
  private RelativeEncoder bottomRightEncoder;
  private RelativeEncoder bottomLeftEncoder;

  private DutyCycleEncoder pivotEncoder;
  private ArmFeedforward pivotFF;
  private SimpleMotorFeedforward flywheelFF;

  /** Creates a new Shooer. */
  public Shooter() {
    pivotMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);

    topRightWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);
    topLeftWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);    
    bottomRightWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);
    bottomLeftWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);

    pivotController = pivotMotor.getPIDController();

    topRightController = topRightWheelMotor.getPIDController();
    topLeftController = topLeftWheelMotor.getPIDController();
    bottomRightController = bottomRightWheelMotor.getPIDController();
    bottomLeftController = bottomLeftWheelMotor.getPIDController();

    pivotEncoder = new DutyCycleEncoder(ShooterConstants.pivotConstants.kAbsoulteEncoderPort);
    topRightEncoder = topRightWheelMotor.getEncoder();
    topLeftEncoder = topLeftWheelMotor.getEncoder();
    bottomRightEncoder = bottomRightWheelMotor.getEncoder();
    bottomLeftEncoder = bottomLeftWheelMotor.getEncoder();

    pivotFF = new ArmFeedforward(
      ShooterConstants.pivotConstants.pivotControllerFFConstants.getS(),
      ShooterConstants.pivotConstants.pivotControllerFFConstants.getV(), 
      ShooterConstants.pivotConstants.pivotControllerFFConstants.getG(), 
      ShooterConstants.pivotConstants.pivotControllerFFConstants.getA());
    
    flywheelFF = new SimpleMotorFeedforward(
      ShooterConstants.flywheelConstants.flywheelControlllerFFConstants.getS(),
      ShooterConstants.flywheelConstants.flywheelControlllerFFConstants.getV(), 
      ShooterConstants.flywheelConstants.flywheelControlllerFFConstants.getG());

    config();
  }

  private void config(){
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.clearFaults();
    pivotMotor.setSmartCurrentLimit(40);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.enableVoltageCompensation(12);
    pivotMotor.burnFlash();

    flywheelConfig(topRightWheelMotor);
    flywheelConfig(topLeftWheelMotor);
    flywheelConfig(bottomRightWheelMotor);
    flywheelConfig(bottomLeftWheelMotor);

    flywheelControllerConfig(topRightController, topRightWheelMotor, topRightEncoder);
    flywheelControllerConfig(bottomRightController, bottomRightWheelMotor, bottomRightEncoder);
    flywheelControllerConfig(topLeftController, topLeftWheelMotor, topLeftEncoder);
    flywheelControllerConfig(bottomLeftController, bottomLeftWheelMotor, bottomLeftEncoder);
    pivotControllerConfig();

    bottomRightWheelMotor.follow(topRightWheelMotor);
    bottomLeftWheelMotor.follow(topLeftWheelMotor);
  }

  private void flywheelConfig(CANSparkMax motor){
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    motor.setSmartCurrentLimit(60);
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(12);
    motor.burnFlash();
  }

  private void flywheelControllerConfig(SparkPIDController controller, CANSparkMax motor, RelativeEncoder encoder){
    controller.setP(ShooterConstants.flywheelConstants.flyWheelControllerConstants.getP());
    controller.setI(ShooterConstants.flywheelConstants.flyWheelControllerConstants.getI());
    controller.setD(ShooterConstants.flywheelConstants.flyWheelControllerConstants.getD());

    controller.setPositionPIDWrappingMinInput(-0.5 * ShooterConstants.flywheelConstants.kWheelGearRatio);
    controller.setPositionPIDWrappingMaxInput(0.5 * ShooterConstants.flywheelConstants.kWheelGearRatio);
    controller.setPositionPIDWrappingEnabled(true);

    controller.setFeedbackDevice(encoder);

    motor.burnFlash();
  }

  private void pivotControllerConfig(){
    pivotController.setP(ShooterConstants.pivotConstants.pivotControllerConstants.getP());
    pivotController.setI(ShooterConstants.pivotConstants.pivotControllerConstants.getI());
    pivotController.setD(ShooterConstants.pivotConstants.pivotControllerConstants.getD());

    pivotController.setPositionPIDWrappingMinInput(-0.5 * ShooterConstants.pivotConstants.kPivotGearRatio);
    pivotController.setPositionPIDWrappingMaxInput(0.5 * ShooterConstants.pivotConstants.kPivotGearRatio);
    pivotController.setPositionPIDWrappingEnabled(true);

    pivotController.setFeedbackDevice((MotorFeedbackSensor) pivotEncoder);

    pivotMotor.burnFlash();
  }

  public void setPivotPosistion(Rotation2d demand){
    pivotController.setReference(
      demand.getRotations(), 
      ControlType.kPosition, 
      0, 
      pivotFF.calculate(0, 0));
  }

  public void setFlywheelVelocity(double demand){
    topLeftController.setReference(
      demand, 
      ControlType.kVelocity, 
      0, 
      flywheelFF.calculate(0, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
