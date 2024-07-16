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

        offset = Rotation2d.fromRotations(constants.offset());

        driveMotor = new CANSparkMax(constants.driveID(), MotorType.kBrushless);
        configureDrive();

        absoluteEncoder = new CANcoder(constants.encoderID());
        configueCANcoder();

        azimuthMotor = new CANSparkMax(constants.azimuthID(), MotorType.kBrushless);
        configureAzimuth();
    }

    @Override
    public void periodic(){
        // Drive Telemetry //
        SmartDashboard.putBoolean(idName + "/Drive Motor/Connected", driveMotor.getLastError().equals(REVLibError.kOk));
        SmartDashboard.putNumber(idName + "/Drive Motor/Current Amperage", getDriveAmps());
        SmartDashboard.putNumber(idName + "/Drive Motor/Distance Meters", getDriveDistanceMeters());
        SmartDashboard.putNumber(idName + "/Drive Motor/TemperatureFarenheit", getDriveTemperature());
        SmartDashboard.putNumber(idName + "/Drive Motor/VelocityMPS", getDriveVelocityMPS());
        SmartDashboard.putNumber(idName + "/Drive Motor/Voltage", getDriveVoltage());

        // Azimuth telemtry //
        SmartDashboard.putBoolean(idName + "/Azimuth Motor/Connected", azimuthMotor.getLastError().equals(REVLibError.kOk));
        SmartDashboard.putNumber(idName + "/Azimuth Motor/Current Amperage", getAzimuthAmps());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/Posistion Meters", getAzimuthPosistion());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/TemperatureFarenheit", getAzimuthTemperature());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/VelocityMPS", getAzimuthVelocity());
        SmartDashboard.putNumber(idName + "/Azimuth Motor/Voltage", getAzimuthVoltage());

        // CANCoder Telemetry // 
        SmartDashboard.putBoolean(idName + "/Absolute Encoder/Connected",  BaseStatusSignal.refreshAll(absolutePositionSignal).isOK());
        SmartDashboard.putNumber(idName + "/Absolute Encoder/Posistion after offset",  getAbsolutePosistion().getRotations());
        SmartDashboard.putNumber(idName + "/Absolute Encoder/Raw Posistion",  absolutePositionSignal.getValueAsDouble());

    }

    /**
     * Configures the drive motor of the swerve module by putting neccessary limits and configurations
     */
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

    /**
     * Configures the azimuth motor of the swerve module by putting neccessary limits and configurations
     */
    private void configureAzimuth(){
        azimuthMotor.restoreFactoryDefaults();
        // This allows the motor some time to set configs // 
        azimuthMotor.setCANTimeout(250);
        azimuthMotor.setInverted(azimuthFlipped);
        azimuthMotor.setSmartCurrentLimit(30);
        azimuthMotor.enableVoltageCompensation(12.0);
        azimuthMotor.setIdleMode(IdleMode.kCoast);

        azimuthEncoder = azimuthMotor.getEncoder();
        resetAzimuthPosistion();
        // This allows for more accurate and up-to-date readings //
        azimuthEncoder.setMeasurementPeriod(20);
        azimuthEncoder.setAverageDepth(2);


        angleController = azimuthMotor.getPIDController();
        setAzimuthPIDController(
            SwerveConstants.angleController.getP(),
            SwerveConstants.angleController.getI(),
            SwerveConstants.angleController.getD());

        // This three lines allow the controller to know the range of its movement // 
        angleController.setPositionPIDWrappingMinInput(-0.5 * SwerveConstants.angleGearRatio);
        angleController.setPositionPIDWrappingMaxInput(0.5 * SwerveConstants.angleGearRatio);
        angleController.setPositionPIDWrappingEnabled(true);

        angleController.setFeedbackDevice(azimuthEncoder);
        azimuthMotor.setCANTimeout(10);
        azimuthMotor.burnFlash();
    }

    /**
     * Configures the CANcoder to update every 50 hertz
     * This allows for more accurate and up-to-date posistional information
     */
    private void configueCANcoder(){
        absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePositionSignal);
        absoluteEncoder.optimizeBusUtilization();
    }

    /** 
     * Sets the relative encoder of the azimuth to the value of the absolute encoder on boot
    */
    public void resetAzimuthPosistion(){
        azimuthEncoder.setPosition(getAbsolutePosistion().getRotations() * SwerveConstants.angleGearRatio);
    }

    /**
     * Sets the desired speed of the drive motor and desired angle of the azimuth for the swerve module
     * @param state the raw desired state that is optimized and then passed into the drive and azimuth motor
     */
    public void setModuleState(SwerveModuleState state){
        // The optimize function allows for a more efficent use of our azimuth
        // Example: if the azimuth needs to move 270 degrees clockwise the optimize state will make sure the azimuth moves 90 degrees counter-clockwise
        // This makes the azimuth never trun more than 90 degrees at once
        state = SwerveModuleState.optimize(state, getModuleState().angle);
        
        // Since the drive motor is being set in percentile output we need to take the desired speed and divide by max speed to get a value between 0 and 1
        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxLinearSpeed);

        setAzimuthPosistion(state);
    }

    /**
     * Uses SparkPIDController to set the posistion of the azimuth motor 
     * @param state contians the optimized desired angle to put the azimuth at
     */
    public void setAzimuthPosistion(SwerveModuleState state){
        // This is to reduce jitter
        // This makes sure that we are not constantly setting the azimuth posistion
        // Stick drift/human error can cause a small demand in azimuth posistion when not needed
        if (Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.maxLinearSpeed * 0.001)) {
            state.angle = lastAngle;
        }
        else {
            state.angle = state.angle;
        }

        // The controller takes in the demand as a rotation for posistional control
        angleController.setReference(
            state.angle.getRotations() * SwerveConstants.angleGearRatio, 
            ControlType.kPosition, 
            0);

        lastAngle = state.angle;
    }

    /** 
     * Sets up the desired constants for the azimuth PID controller
     * @param p desired kP constant
     * @param i desired kI constant
     * @param d desired kD constant
     */
    public void setAzimuthPIDController(double p, double i, double d){
        angleController.setP(p);
        angleController.setI(i);
        angleController.setD(d);
    }

    /**
     * Stops the swerve module if neccessary
     */
    public void stop(){
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    /**
     * Returns the state of the module
     * @return returns the drive velocity in metters per second and the azimuth posistion in rotations in a SwerveModuleState
     */
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocityMPS(), Rotation2d.fromRotations(getAzimuthPosistion()));
    }

    /**
     * Set drive motor voltage
     * @param demand amount of voltage between -12 and 12 to set to the motor
     */
    public void setDriveVoltage(double demand){
        driveMotor.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    /**
     * Get the posistion of the drive wheel
     * @return returns the posistion of the drive wheel in rotations
     */
    public double getDrivePosistion(){
        return driveEncoder.getPosition() / SwerveConstants.driveGearRatio;
    }

    /**
     * Get the velocity of the drive motor in MPS
     * @return returns the velocity in meters per second
     */
    public double getDriveVelocityMPS(){
        return (driveEncoder.getVelocity() * SwerveConstants.wheelCircumference) / (60.0 * SwerveConstants.driveGearRatio);
    }

    /**
     * Get the temperature of the drive motor in Farenheit
     * @return returns the temperature of the motor
     */
    public double getDriveTemperature(){
        return (driveMotor.getMotorTemperature() * (9.0/5.0)) + 32;
    }

    /**
     * Get the current voltage of the drive motor
     * @return returns the amount of voltage on the drive motor
     */
    public double getDriveVoltage(){
        return (driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    }

    /**
     * Get the current amperage of the drive motor
     * @return returns the amount of amps on the drive motor
     */
    public double getDriveAmps(){
        return driveMotor.getOutputCurrent();
    }

    /**
     * Get the distance the drive motor traveled
     * @return returns the distance traveled in meters
     */
    public double getDriveDistanceMeters(){
        return (driveEncoder.getPosition() * SwerveConstants.wheelCircumference) / SwerveConstants.driveGearRatio;
    }

    /**
     * Set azimuth motor voltage
     * @param demand amount of voltage between -12 and 12 to set to the motor
     */
    public void setAzimuthVoltage(double demand){
        azimuthMotor.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    /**
     * Get the posistion of the azimuth wheel
     * @return returns the posistion of the azimuth wheel in rotations
     */
    public double getAzimuthPosistion(){
        return azimuthEncoder.getPosition() / SwerveConstants.angleGearRatio;
    }

    /**
     * Get the velocity of the azimuth motor in RPS
     * @return returns the velocity in rotations per second
     */
    public double getAzimuthVelocity(){
        return (azimuthEncoder.getVelocity()) / (SwerveConstants.angleGearRatio * 60) ;
    }

    /**
     * Get the temperature of the azimuth motor in Farenheit
     * @return returns the temperature of the motor
     */
    public double getAzimuthTemperature(){
        return (azimuthMotor.getMotorTemperature() * (9.0/5.0)) + 32;
    }

    /**
     * Get the current voltage of the azimuth motor
     * @return returns the amount of voltage on the drive motor
     */
    public double getAzimuthVoltage(){
        return (azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage());
    }

    /**
     * Get the current amperage of the azimuth motor
     * @return returns the amount of amps on the drive motor
     */
    public double getAzimuthAmps(){
        return azimuthMotor.getOutputCurrent();
    }

    /**
     * Get the posiston of the CANCoder
     * @return returns the posistion of the cancoder from -0.5 to 0.5 rotations
     */
    public Rotation2d getAbsolutePosistion(){
        double val = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(val).minus(offset);

        return angle;
    }

    /**
     * @return number of the swerve module
     */
    public int getNum(){
        return num;
    }

    /**
     * @return name of the swerve module
     */
    public String getName(){
        return idName;
    }
}
