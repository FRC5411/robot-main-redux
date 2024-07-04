// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class SwerveModule extends SubsystemBase{
    public String idName;
    private CANSparkMax azimuthMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder azimuthEncoder;
    private CANcoder absoluteEncoder;

    private Rotation2d offset;

    private final SparkPIDController angleController;

    public static LoggedTunableNumber kAzimuthP;
    public static LoggedTunableNumber kAzimuthD;

    public ModuleConstants constants;


    public SwerveModule(String idName, ModuleConstants constants){
        this.idName = idName;
        this.constants = constants;

        azimuthMotor = new CANSparkMax(constants.azimuthID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(constants.driveID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        azimuthEncoder = azimuthMotor.getEncoder();
        absoluteEncoder = new CANcoder(constants.encoderID);
        offset = new Rotation2d(constants.offset);

        // TODO: Tune angle controller
        angleController = azimuthMotor.getPIDController();
        configAngleController();

        kAzimuthP = new LoggedTunableNumber("Module " + idName + " Azimuth P", angleController.getP());
        kAzimuthD = new LoggedTunableNumber("Module " + idName + " Azimuth D", angleController.getD());

        config();

    }

    private void config(){
        azimuthMotor.restoreFactoryDefaults();
        azimuthMotor.setSmartCurrentLimit(30);
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

    private void configAngleController(){
        angleController.setP(constants.angleController.getP());
        angleController.setI(constants.angleController.getI());
        angleController.setD(constants.angleController.getD());

        angleController.setPositionPIDWrappingMinInput(-0.5 * SwerveConstants.angleGearRatio);
        angleController.setPositionPIDWrappingMaxInput(0.5 * SwerveConstants.angleGearRatio);
        angleController.setPositionPIDWrappingEnabled(true);

        angleController.setFeedbackDevice(azimuthEncoder);

        azimuthMotor.burnFlash();
    }

    public double getAbsoultePosistion(){
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() - offset.getDegrees();

        if(constants.absoulteFlipped){return angle * -1;}

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
        azimuthEncoder.setPosition(getAbsoultePosistion());
    }

    public void setAzimuthPIDController(double p, double i, double d){
        angleController.setP(p);
        angleController.setI(i);
        angleController.setD(d);
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
        setAzimuthPosistion(state.angle);

    }

    public void setAzimuthPosistion(Rotation2d demand){
        // The controller takes in the demand as a rotation
        angleController.setReference(demand.getRotations(), ControlType.kPosition, 0);
    }

    public void setDriveVoltage(double demand){
        driveMotor.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    public void stop(){
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAzimuthPosistion()));
    }

    public double getDistance(){
        //TODO: Check if correct
        return driveEncoder.getPosition() / driveEncoder.getCountsPerRevolution() * (Math.PI * SwerveConstants.wheelDiameter);
    }

    public void periodic(){
        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {setAzimuthPIDController(kAzimuthP.get(), (0), kAzimuthD.get());}, 
            kAzimuthP, kAzimuthD);
    }
}
