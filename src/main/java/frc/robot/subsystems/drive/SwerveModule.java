// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SwerveModule extends SubsystemBase{

    private CANSparkMax azimuthMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder azimuthEncoder;

    private boolean driveFlipped;
    private boolean azimuthFlipped;
    private boolean absoluteFlipped;

    private CANcoder absoluteEncoder;
    private StatusSignal<Double> absolutePositionSignal;

    private Rotation2d offset;

    private SparkPIDController angleController;

    public ModuleConstants constants;

    private int num;
    private String idName;

    private Rotation2d lastAngle;

    public SwerveModule(String idName, int num, ModuleConstants constants){
        this.idName = idName;
        this.num = num;

        driveMotor = new CANSparkMax(constants.driveID(), MotorType.kBrushless);
        configureDrive();

        absoluteEncoder = new CANcoder(constants.encoderID());
        configueCANcoder();

        azimuthMotor = new CANSparkMax(constants.azimuthID(), MotorType.kBrushless);
        configureAzimuth();

        offset = Rotation2d.fromRotations(constants.offset());

    }

    @Override
    public void periodic(){
        // Drive Telemetry //
        SmartDashboard.putBoolean(idName + "/Drive Motor/Connected", driveMotor.getLastError().equals(REVLibError.kOk));
        SmartDashboard.putNumber(idName + "/Drive Motor/Current Amperage", getDriveAmps());
        SmartDashboard.putNumber(idName + "/Drive Motor/Distance Meters", getDriveDistanceMeters());
        SmartDashboard.putNumber(idName + "/Drive Motor/TemperatureFarenheit", getDriveTemperature());
        SmartDashboard.putNumber(idName + "/Drive Motor/VelocityMPS", getDriveVelocity());
        SmartDashboard.putNumber(idName + "/Drive Motor/Voltage", getDriveVoltage());

        // Azimuth telemtry //
        SmartDashboard.putBoolean(idName + "/Azimuth Motor/Connected", azimuthMotor.getLastError().equals(REVLibError.kOk));
        SmartDashboard.putNumber(idName + "/Azimuth Motor/Current Amperage", getAzimuthAmps());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/Posistion Meters", getAzimuthPosistion());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/TemperatureFarenheit", getAzimuthTemperature());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/VelocityMPS", getAzimuthVelocity());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/Voltage", getAzimuthVoltage());

        // CANcoder Telemetry // 
        SmartDashboard.putBoolean(idName + "/Absolute Encoder/Connected",  BaseStatusSignal.refreshAll(absolutePositionSignal).isOK());
        SmartDashboard.putNumber(idName + "/Absolute Encoder/Posistion after offset",  getAbsolutePosistion().getRotations());
        SmartDashboard.putNumber(idName + "/Absolute Encoder/Raw Posistion",  absolutePositionSignal.getValueAsDouble());

    }

    private void configureDrive(){
        driveMotor.restoreFactoryDefaults();
        driveMotor.setCANTimeout(250);
        driveMotor.setInverted(driveFlipped);
        driveMotor.setSmartCurrentLimit(60);
        driveMotor.enableVoltageCompensation(12.0);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(20);
        driveEncoder.setAverageDepth(4);
        driveMotor.setCANTimeout(10);
        driveMotor.burnFlash();
    }

    private void configureAzimuth(){
        azimuthMotor.restoreFactoryDefaults();
        azimuthMotor.setCANTimeout(250);
        azimuthMotor.setInverted(azimuthFlipped);
        azimuthMotor.setSmartCurrentLimit(30);
        azimuthMotor.enableVoltageCompensation(12.0);
        azimuthMotor.setIdleMode(IdleMode.kCoast);

        azimuthEncoder = azimuthMotor.getEncoder();
        resetAzimuthPosistion();
        azimuthEncoder.setMeasurementPeriod(20);
        azimuthEncoder.setAverageDepth(2);


        angleController = azimuthMotor.getPIDController();
        setAzimuthPIDController(
            SwerveConstants.angleController.getP(),
            SwerveConstants.angleController.getI(),
            SwerveConstants.angleController.getD());

        angleController.setPositionPIDWrappingMinInput(-0.5 * SwerveConstants.angleGearRatio);
        angleController.setPositionPIDWrappingMaxInput(0.5 * SwerveConstants.angleGearRatio);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setFeedbackDevice(azimuthEncoder);
        azimuthMotor.setCANTimeout(10);
        azimuthMotor.burnFlash();
    }

    private void configueCANcoder(){
        absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePositionSignal);
        absoluteEncoder.optimizeBusUtilization();
    }

    public void resetAzimuthPosistion(){
        azimuthEncoder.setPosition(getAbsolutePosistion().getRotations() * SwerveConstants.angleGearRatio);
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

    public void setAzimuthPIDController(double p, double i, double d){
        angleController.setP(p);
        angleController.setI(i);
        angleController.setD(d);
    }

    public void stop(){
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAzimuthPosistion()));
    }

    public void setDriveVoltage(double demand){
        driveMotor.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    public void setAzimuthVoltage(double demand){
        azimuthMotor.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    public double getDrivePosistion(){
        return driveEncoder.getPosition() / SwerveConstants.angleGearRatio;
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getDriveTemperature(){
        return (driveMotor.getMotorTemperature() * (9.0/5.0)) + 32;
    }

    public double getDriveVoltage(){
        return (driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    }

    public double getDriveAmps(){
        return driveMotor.getAppliedOutput();
    }

    public double getDriveDistanceMeters(){
        return (driveEncoder.getPosition() * SwerveConstants.wheelCircumference) / SwerveConstants.driveGearRatio;
    }

    public double getAzimuthPosistion(){
        return azimuthEncoder.getPosition() / SwerveConstants.angleGearRatio;
    }

    public double getAzimuthVelocity(){
        return azimuthEncoder.getVelocity();
    }

    public double getAzimuthTemperature(){
        return (azimuthMotor.getMotorTemperature() * (9.0/5.0)) + 32;
    }

    public double getAzimuthVoltage(){
        return (azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage());
    }

    public double getAzimuthAmps(){
        return azimuthMotor.getAppliedOutput();
    }

    public Rotation2d getAbsolutePosistion(){
        double val = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(val).minus(offset);

        return angle;
    }

    public int getNum(){
        return num;
    }

    public String getName(){
        return idName;
    }
}
