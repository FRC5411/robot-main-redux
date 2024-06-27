// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    public String idName;
    private CANSparkMax azimuthMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder azimuthEncoder;
    private CANcoder absoluteEncoder;

    private Rotation2d offset;

    private final PIDController angleController;

    public SwerveModule(String idName, ModuleConstants constants){
        this.idName = idName;

        azimuthMotor = new CANSparkMax(constants.azimuthID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(constants.driveID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        azimuthEncoder = azimuthMotor.getEncoder();
        absoluteEncoder = new CANcoder(constants.encoderID);
        offset = new Rotation2d(constants.offset);
        // TODO: Tune angle controller
        angleController = new PIDController(0.01, 0, 0);
        config();

    }

    private void config(){
        azimuthMotor.restoreFactoryDefaults();
        azimuthMotor.setSmartCurrentLimit(20);
        azimuthMotor.setIdleMode(IdleMode.kCoast);
        azimuthMotor.enableVoltageCompensation(12);
        azimuthEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);

        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.enableVoltageCompensation(12);
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);

        absoluteEncoder.clearStickyFaults();

        azimuthMotor.burnFlash();
        driveMotor.burnFlash();

        resetAzimuthPosistion();

    }

    public double getAbsoultePosistion(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() - offset.getDegrees();
    }

    public void resetAzimuthPosistion(){
        azimuthEncoder.setPosition(getAbsoultePosistion());
    }

    public double getDrivePosistion(){
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getAzimuthPosistion(){
        return azimuthEncoder.getPosition();
    }


    public void setModuleState(SwerveModuleState state){
        // Will stop any jittering from occuring in the swerve wheels // 
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            // return to make sure we exit the program // 
            return;
        }

        state = SwerveModuleState.optimize(state, getModuleState().angle);

        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxLinearSpeed);
        azimuthMotor.set(angleController.calculate(getAzimuthPosistion(), state.angle.getRadians()));

    }

    public void stop(){
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    public CANcoder getCANCoder(){
        return absoluteEncoder;
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAzimuthPosistion()));
    }

    public double getDistance(){
        //TODO: Check if correct
        return driveEncoder.getPosition() / driveEncoder.getCountsPerRevolution() * (Math.PI * SwerveConstants.wheelDiameter);
    }
}
