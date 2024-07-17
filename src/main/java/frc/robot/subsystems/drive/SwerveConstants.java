// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants{
    // These three values need to be found from the CAD // 
    public static final double trackWidth = 0.61595;
    public static final double wheelBase = 0.61595;
    public static final double wheelDiameter = Units.inchesToMeters(3.95); // 3.8897

    public static final double robotRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    public static final double wheelCircumference = wheelDiameter * Math.PI;

    // This ID will need to be found
    public static final int pigeonID = 10;
    
    // This can be found online when you search up 'mk4i swerve module'
    // Then look for the module name and find the gear ratio and linear speed
    public static final double driveGearRatio = 5.357 / 1.0; // L2 Ratio
    public static final double angleGearRatio = 150.0 / 7.0;
    public static final double maxLinearSpeed  = 4.8;

    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    public static final double maxRotationalVelocity = maxLinearSpeed / robotRadius;

    // These values needs to be tuned // 
    public static PIDController angleController = new PIDController(0.1, 0, 0);

    // These values need to be found // 
    public final class FLModule{
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.584717);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset, 
            true, 
            false, 
            false);
    }

    // These values need to be found // 
    public final class FRModule{
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.823486);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset, 
            true,
            false, 
            false);
    }

    // These values need to be found // 
    public final class BLModule{
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.291992);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset, 
            true,
            false, 
            false);
    }

    // These values need to be found // 
    public final class BRModule{
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.416504);
      public static final ModuleConstants constants =
          new ModuleConstants(
            angleMotorID, 
            driveMotorID, 
            canCoderID, 
            angleOffset, 
            true,
            false, 
            false);
    }
    
}   