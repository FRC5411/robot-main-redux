// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private StatusSignal<Double> absolutePositionSignal;

    private Rotation2d offset;

    private SparkPIDController angleController;

    public ModuleConstants constants;

    private int num;

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
            SwerveConstants.angleP,
            0,
            0);
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

    public Rotation2d getAbsolutePosistion(){
        double val = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        SmartDashboard.putNumber("Drive/Val", val);
        Rotation2d angle = Rotation2d.fromRotations(val).minus(offset);

        return angle;
    }

    public double getDrivePosistion(){
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getAzimuthPosistion(){
        return azimuthEncoder.getPosition() / SwerveConstants.angleGearRatio;
    }

    public CANcoder getCANCoder(){
        return absoluteEncoder;
    }

    public void resetAzimuthPosistion(){
        azimuthEncoder.setPosition(getAbsolutePosistion().getDegrees() * SwerveConstants.angleGearRatio);
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

    public int getNum(){
        return num;
    }

    public void periodic(){}
}
