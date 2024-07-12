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

    modules = new SwerveModule[]{
      FL = new SwerveModule("Front Left", 0, SwerveConstants.FLModule.constants),
      FR = new SwerveModule("Front Right", 1, SwerveConstants.FRModule.constants),
      BL = new SwerveModule("Back Left", 2, SwerveConstants.BLModule.constants),
      BR = new SwerveModule("Back Right", 3, SwerveConstants.BRModule.constants)
    };

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

    for(SwerveModule mod : modules){
      states[mod.getNum()] = new SwerveModulePosition(
        mod.getDistance(), 
        new Rotation2d(mod.getAzimuthPosistion()));
    }

    return states;
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void moveDriveVolts(double demand){
    for(SwerveModule mod: modules){
      mod.setDriveVoltage(demand);
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

    for(SwerveModule mod: modules){
      mod.setModuleState(swerveModuleStates[mod.getNum()]);
    }
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule mod: modules){
      states[mod.getNum()] = mod.getModuleState();
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

    SmartDashboard.putNumber("Module / FR / ABSOULTE POS", FR.getAbsolutePosistion().getDegrees());
    SmartDashboard.putNumber("Module / FR / RELATIVE POS", FR.getAzimuthPosistion());

    SmartDashboard.putNumber("Module / FL / ABSOULTE POS", FL.getAbsolutePosistion().getDegrees());
    SmartDashboard.putNumber("Module / FL / RELATIVE POS", FL.getAzimuthPosistion());

    SmartDashboard.putNumber("Module / BR / ABSOULTE POS", BR.getAbsolutePosistion().getDegrees());
    SmartDashboard.putNumber("Module / BR / RELATIVE POS", BR.getAzimuthPosistion());

    SmartDashboard.putNumber("Module / BL / ABSOULTE POS", BL.getAbsolutePosistion().getDegrees());
    SmartDashboard.putNumber("Module / BL / RELATIVE POS", BL.getAzimuthPosistion());

    SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());

    SmartDashboard.putNumber("Pose Estimate Yaw", odometry.getPoseMeters().getRotation().getDegrees());
  }


}
