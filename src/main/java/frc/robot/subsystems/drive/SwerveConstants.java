// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants{
    public static final double trackWidth = Units.inchesToMeters(27);
    public static final double wheelBase = Units.inchesToMeters(27);
    public static final double wheelDiameter = Units.inchesToMeters(3.95); // 3.8897

    public static final double robotRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final int pigeonID = 10;
    
    public static final double driveGearRatio = 6.75 / 1.0; // L2 Ratio
    public static final double angleGearRatio = 150.0 / 7.0;

    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    public static final double maxLinearSpeed  = 4.60248;
    public static final double maxRotationalVelocity = maxLinearSpeed / robotRadius;


    public static double kp = 0.01;

    public final class FLModule{
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.499);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset.getDegrees(), 
            false,
            false, 
            false);
    }

    public final class FRModule{
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.32);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset.getDegrees(), 
            false,
            false, 
            false);
    }

    public final class BLModule{
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.165);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset.getDegrees(), 
            false,
            false, 
            false);
    }

    public final class BRModule{
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.171);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset.getDegrees(), 
            false,
            false, 
            false);
    }
    
}   