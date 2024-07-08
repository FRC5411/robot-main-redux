// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax pivotMotor;
  // TODO: Find a better name gang
  private CANSparkMax topRightWheelMotor;
  private CANSparkMax topLeftWheelMotor;
  private CANSparkMax bottomRightWheelMotor;
  private CANSparkMax bottomLeftWheelMotor;

  private SparkPIDController topRightController;
  private SparkPIDController topLeftController;
  private SparkPIDController bottomRightController;
  private SparkPIDController bottomLeftController;

  private RelativeEncoder topRightEncoder;
  private RelativeEncoder topLeftEncoder;
  private RelativeEncoder bottomRightEncoder;
  private RelativeEncoder bottomLeftEncoder;

  /** Creates a new Shooer. */
  public Shooter() {
    pivotMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);

    topRightWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);
    topLeftWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);    
    bottomRightWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);
    bottomLeftWheelMotor = new CANSparkMax(ShooterConstants.pivotConstants.kPivotPort, MotorType.kBrushless);

    topRightController = topRightWheelMotor.getPIDController();
    topLeftController = topLeftWheelMotor.getPIDController();
    bottomRightController = bottomRightWheelMotor.getPIDController();
    bottomLeftController = bottomLeftWheelMotor.getPIDController();

    topRightEncoder = topRightWheelMotor.getEncoder();
    topLeftEncoder = topLeftWheelMotor.getEncoder();
    bottomRightEncoder = bottomRightWheelMotor.getEncoder();
    bottomLeftEncoder = bottomLeftWheelMotor.getEncoder();
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
    controller.setP(ShooterConstants.flywheelConstants.flyWheelConstants.getP());
    controller.setI(ShooterConstants.flywheelConstants.flyWheelConstants.getI());
    controller.setD(ShooterConstants.flywheelConstants.flyWheelConstants.getD());

    controller.setPositionPIDWrappingMinInput(-0.5 * ShooterConstants.flywheelConstants.kWheelGearRatio);
    controller.setPositionPIDWrappingMaxInput(0.5 * ShooterConstants.flywheelConstants.kWheelGearRatio);
    controller.setPositionPIDWrappingEnabled(true);

    controller.setFeedbackDevice(encoder);

    motor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
