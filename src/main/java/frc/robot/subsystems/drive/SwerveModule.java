// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SwerveModule extends SubsystemBase{
    public String idName;
    private CANSparkMax azimuthMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder azimuthEncoder;

    private boolean driveFlipped;
    private boolean azimuthFlipped;
    private boolean absoluteFlipped;

    private CANcoder absoluteEncoder;

    private Rotation2d offset;

    private final SparkPIDController angleController;

    public ModuleConstants constants;

    private Rotation2d lastAngle;

    public SwerveModule(String idName, ModuleConstants constants){
        this.idName = idName;

        azimuthMotor = new CANSparkMax(constants.azimuthID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(constants.driveID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        azimuthEncoder = azimuthMotor.getEncoder();
        absoluteEncoder = new CANcoder(constants.encoderID);
        offset = new Rotation2d(constants.offset);
        driveFlipped = constants.driveFlipped;
        azimuthFlipped = constants.azimuthFlipped;
        absoluteFlipped = constants.absoluteFlipped;

        angleController = azimuthMotor.getPIDController();

        config();
        lastAngle = Rotation2d.fromDegrees(azimuthEncoder.getPosition());
    }

    private void config(){
        azimuthMotor.restoreFactoryDefaults();
        azimuthMotor.setSmartCurrentLimit(30);
        azimuthMotor.setIdleMode(IdleMode.kCoast);
        azimuthMotor.enableVoltageCompensation(12);
        azimuthMotor.setInverted(azimuthFlipped);
        azimuthEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
        azimuthEncoder.setMeasurementPeriod(20);

        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.enableVoltageCompensation(12);
        driveMotor.setInverted(driveFlipped);
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(20);

        absoluteEncoder.clearStickyFaults();

        angleController.setP(SwerveConstants.angleP);
        angleController.setI(0);
        angleController.setD(0);

        angleController.setFeedbackDevice(azimuthEncoder);

        azimuthMotor.burnFlash();
        driveMotor.burnFlash();

        resetAzimuthPosistion();

    }

    public Rotation2d getAbsoultePosistion(){
        double val = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

        val = absoluteFlipped == true ? val * -1 : val;

        Rotation2d angle = Rotation2d.fromRotations(val).plus(offset);
        return angle;
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

    public CANcoder getCANCoder(){
        return absoluteEncoder;
    }

    public void resetAzimuthPosistion(){
        azimuthEncoder.setPosition(getAbsoultePosistion().getDegrees());
    }

    public void setAzimuthPIDController(double p, double i, double d){
        angleController.setP(p);
        angleController.setI(i);
        angleController.setD(d);
    }


    public void setModuleState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getModuleState().angle);

        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxLinearSpeed);

        setAzimuthPosistion(state);
    }

    public void setAzimuthPosistion(SwerveModuleState state){
        Rotation2d desiredAngle;
        if (Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.maxLinearSpeed * 0.001)) {
            desiredAngle = lastAngle;
        }
        else {
            desiredAngle = state.angle;
        }

        // The controller takes in the demand as a rotation
        angleController.setReference(desiredAngle.getDegrees(), ControlType.kPosition, 0);

        lastAngle = desiredAngle;
    }

    public void setDriveVoltage(double demand){
        driveMotor.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    public void stop(){
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAzimuthPosistion()));
    }

    public double getDistance(){
        //TODO: Check if correct
        return driveEncoder.getPosition() / driveEncoder.getCountsPerRevolution() * (Math.PI * SwerveConstants.wheelDiameter);
    }

    public void periodic(){}
}
