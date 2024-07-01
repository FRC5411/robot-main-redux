// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private static LoggedTunableNumber kDriftRate;



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

    kDriftRate = new LoggedTunableNumber("Drift Rate", SwerveConstants.driftRate);
  }

  public Rotation2d getYaw(){
    return new Rotation2d(gyro.getYaw().getValueAsDouble());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];

    for(int i = 0; i < 4; i++){
      states[i] = new SwerveModulePosition(modules[i].getDistance(), new Rotation2d(modules[i].getAzimuthPosistion()));
    }

    return states;
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    final SwerveModuleState[] swerveModuleStates;
      if(fieldRelative){
        swerveModuleStates = kinematics.toSwerveModuleStates(
          discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw())));
      }

      else{
        swerveModuleStates = kinematics.toSwerveModuleStates(
          discretize(new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation)));
      }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxLinearSpeed);

    for(int i = 0; i < 4; i++){
      modules[i].setModuleState(swerveModuleStates[i]);
    }
  }

  private ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * dt * kDriftRate.get()));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
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

  @Override
  public void periodic() {
    odometry.update(getYaw(), getSwerveModulePositions());

    for (SwerveModule mod : modules) {
      SmartDashboard.putNumber("Swerve Module " + mod.idName + " Cancoder Posistion: ", mod.getCANCoder().getAbsolutePosition().getValueAsDouble());
      SmartDashboard.putNumber("Swerve Module " + mod.idName + " Azimuth Posistion: ", mod.getModuleState().angle.getDegrees());
      SmartDashboard.putNumber("Swerve Module " + mod.idName + " Drive Velocity", mod.getModuleState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("Robot Yaw: ", getYaw().getDegrees());

    field2d.setRobotPose(getPose());

    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> {kDriftRate.get();}, 
      kDriftRate);


  }
}
