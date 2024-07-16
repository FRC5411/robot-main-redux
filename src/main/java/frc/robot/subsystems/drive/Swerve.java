// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.debugging.LoggedTunableNumber;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  private final SwerveModule FR;
  private final SwerveModule FL;
  private final SwerveModule BR;
  private final SwerveModule BL;

  private final Pigeon2 gyro;
  private final StatusSignal<Double> gyroYaw;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final SwerveModule[] modules;
  private final Field2d field2d;

  private static LoggedTunableNumber kAzimuthP;


  public Swerve() {
    FL = new SwerveModule("Front Left", 0, SwerveConstants.FLModule.constants);
    FR = new SwerveModule("Front Right", 1, SwerveConstants.FRModule.constants);
    BL = new SwerveModule("Back Left", 2, SwerveConstants.BLModule.constants);
    BR = new SwerveModule("Back Right", 3, SwerveConstants.BRModule.constants);

    gyro = new Pigeon2(SwerveConstants.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0);
    gyroYaw = gyro.getYaw();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, gyroYaw);
    gyro.optimizeBusUtilization();

    modules = new SwerveModule[]{FL, FR, BL, BR};

    kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
      new Translation2d(SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0),
      new Translation2d(-SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
      new Translation2d(-SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0));
    
    odometry = new SwerveDriveOdometry(kinematics, getYaw(), getSwerveModulePositions());

    field2d = new Field2d();

    SmartDashboard.putData(field2d);

    kAzimuthP = new LoggedTunableNumber("Azimuth P", SwerveConstants.angleController.getP());
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getSwerveModulePositions());

    field2d.setRobotPose(getPoseMeters());

    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> {setAzimuthP(kAzimuthP.get());}, 
      kAzimuthP);

    SmartDashboard.putNumber("Drive/Gyro Yaw", getYaw().getDegrees());
    SmartDashboard.putBoolean("Drive/Gyro Connected", BaseStatusSignal.refreshAll(gyroYaw).isOK());
    SmartDashboard.putNumber("Drive/Pose Estimate Yaw", odometry.getPoseMeters().getRotation().getDegrees());
  }

  /**
   * Sets all the swerve modules to the required posistion and velocity to be able to carry out the demands of the controller
   * Field relative means the robot will always move forward in the same direction regardless of heading compared to robotrelative which is the oppposite
   * This function gets its inputs from the joystick and is then fed into each swerve module 
   * @param translation : the demand of how much we want to move the robot right, left, forward, and back
   * @param rotation : the demand of how much we want to rotate the robot
   * @param fieldRelative : if you want the robot to be field oriented or robot oriented
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates;

    // This is for if we want field relative drive // 
      if(fieldRelative){
        swerveModuleStates = kinematics.toSwerveModuleStates(
          (ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw())));
      }

      // This is for if we want robot relative drive //
      else{
        swerveModuleStates = kinematics.toSwerveModuleStates(
          (new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation)));
      }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxLinearSpeed);

    for(int i = 0; i < 4; i++){
      modules[i].setModuleState(swerveModuleStates[i]);
    }
  }

  /**
   * Returns the distance driven by the drive motor and the azimuth posistion for each module
   * @return an array of each SwerveModulePosistion in order of FL, FR, BL, and BR
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];

    for(SwerveModule mod : modules){
      states[mod.getNum()] = new SwerveModulePosition(
        mod.getDriveDistanceMeters(), 
        new Rotation2d(mod.getAzimuthPosistion()));
    }

    return states;
  }

  /**
   * Returns the velocity of the drive motor and the azimuth posistion for each module
   * @return An array of each SwerveModuleState in order of FL, FR, BL, and BR
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule mod: modules){
      states[mod.getNum()] = mod.getModuleState();
    }

    return states;
  }

  /**
   * Will reset the odometry of the robot
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
  }

  /**
   * Will move the drive motor of each swerve module by the demand passed in
   * This method is used to find the 'static friction' or kS for Feed forward on drive
   * @param demand amount of voltage passed on to each module
   */
  public void moveDriveVolts(double demand){
    for(SwerveModule mod: modules){
      mod.setDriveVoltage(demand);
    }
  }

  /**
   * Sets the gyro yaw to 0 
   */
  public void zeroHeading(){
    gyro.getConfigurator().setYaw(0);
  }

  /**
   * Sets the kP value of the azimuth controller
   * @param val value for kP for the azimuth controler
   */
  public void setAzimuthP(double val){
    for(SwerveModule mod : modules){
      mod.setAzimuthPIDController(val, 0, 0);
    }
  }

  /**
   * This method returns the current rotational posistion of the robot or the 'yaw'
   * @return get the yaw of the robot from the gyro
   */
  public Rotation2d getYaw(){
    return new Rotation2d(gyroYaw.getValueAsDouble());
  }

  /**
   * This method gets the estimaged pose of the robot
   * @return estimated pose of the robot in meters
   */
  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

}
