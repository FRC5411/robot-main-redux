// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

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
import frc.robot.util.LoggedTunableNumber;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  private final SwerveModule FR;
  private final SwerveModule FL;
  private final SwerveModule BR;
  private final SwerveModule BL;

  private final Pigeon2 gyro;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final SwerveModule[] modules;
  private final Field2d field2d;

  private static LoggedTunableNumber kAzimuthP;

  public Swerve() {
    gyro = new Pigeon2(SwerveConstants.pigeonID);
    gyro.clearStickyFaults();
    zeroHeading();

    FL = new SwerveModule("Front Left", SwerveConstants.FLModule.constants);
    FR = new SwerveModule("Front Right", SwerveConstants.FRModule.constants);
    BL = new SwerveModule("Back Left", SwerveConstants.BLModule.constants);
    BR = new SwerveModule("Back Right", SwerveConstants.BRModule.constants);

    modules = new SwerveModule[]{FL, FR, BL, BR};

    kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
      new Translation2d(SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0),
      new Translation2d(-SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
      new Translation2d(-SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0));
    
    odometry = new SwerveDriveOdometry(kinematics, getYaw(), getSwerveModulePositions());

    field2d = new Field2d();

    SmartDashboard.putData(field2d);

    kAzimuthP = new LoggedTunableNumber("Azimuth P", SwerveConstants.angleP);
  }

  public Rotation2d getYaw(){
    return new Rotation2d(gyro.getYaw().getValueAsDouble());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];

    for(int i = 0; i < 4; i++){
      states[i] = new SwerveModulePosition(
        modules[i].getDistance(), 
        new Rotation2d(modules[i].getAzimuthPosistion()));
    }

    return states;
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void moveDriveVolts(double demand){
    for(int i = 0; i < 4; i++){
      modules[i].setDriveVoltage(demand);
    }
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    final SwerveModuleState[] swerveModuleStates;
      if(fieldRelative){
        swerveModuleStates = kinematics.toSwerveModuleStates(
          (ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw())));
      }

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

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for(int i = 0; i < 4; i++){
      states[i] = modules[i].getModuleState();
    }

    return states;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
  }

  public void setAzimuthP(double val){
    SwerveConstants.angleP = val;
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getSwerveModulePositions());

    field2d.setRobotPose(getPose());

    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> {setAzimuthP(kAzimuthP.get());}, 
      kAzimuthP);

    SmartDashboard.putNumber("Module / FR / ABSOULTE POS", FR.getAbsoultePosistion().getDegrees());
    SmartDashboard.putNumber("Module / FR / RELATIVE POS", FR.getAzimuthPosistion());

    SmartDashboard.putNumber("Module / FL / ABSOULTE POS", FL.getAbsoultePosistion().getDegrees());
    SmartDashboard.putNumber("Module / FL / RELATIVE POS", FL.getAzimuthPosistion());

    SmartDashboard.putNumber("Module / BR / ABSOULTE POS", BR.getAbsoultePosistion().getDegrees());
    SmartDashboard.putNumber("Module / BR / RELATIVE POS", BR.getAzimuthPosistion());

    SmartDashboard.putNumber("Module / BL / ABSOULTE POS", BL.getAbsoultePosistion().getDegrees());
    SmartDashboard.putNumber("Module / BL / RELATIVE POS", BL.getAzimuthPosistion());

    SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());

    SmartDashboard.putNumber("Pose Estimate Yaw", odometry.getPoseMeters().getRotation().getDegrees());
  }


}
