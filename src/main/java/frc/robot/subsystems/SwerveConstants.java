// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants{
  // TODO: Find out these measurements from zesty CAD team
    public static final double trackWidth = Units.inchesToMeters(22);
    public static final double wheelBase = Units.inchesToMeters(22);
    public static final double wheelDiameter = Units.inchesToMeters(3.95); // 3.8897

    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final int pigeonID = 10;
    
    //TODO: Find Gear Ratios
    public static final double driveGearRatio = 6.75 / 1.0; // L2 Ratio
    public static final double angleGearRatio = 150.0 / 7.0;

    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    // TODO: Find accurate max linear speed as this is a rough estimate
    //Linear Speed (meters /sec) = motor speed (RPM) / gear ratio * pi * wheel diameter (meters) / 60
    public static final int maxLinearSpeed  = 5;

    public static final int driftRate = 1;


    //TODO: ID all of the modules and find offsets
    public final class FLModule{
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(53.58);
      public static final PIDController angleController = new PIDController(0.01, 0, 0);
      public static final ModuleConstants constants =
          new ModuleConstants(angleMotorID, driveMotorID, canCoderID, angleOffset.getDegrees(), angleController,false, false, false);
    }

    public final class FRModule{
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-19.78);
      public static final PIDController angleController = new PIDController(0.01, 0, 0);
      public static final ModuleConstants constants =
          new ModuleConstants(angleMotorID, driveMotorID, canCoderID, angleOffset.getDegrees(), angleController, false, false, false);
    }

    public final class BLModule{
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-41.07);
      public static final PIDController angleController = new PIDController(0.01, 0, 0);
      public static final ModuleConstants constants =
          new ModuleConstants(angleMotorID, driveMotorID, canCoderID, angleOffset.getDegrees(), angleController, false, false, false);
    }

    public final class BRModule{
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(90.93);
      public static final PIDController angleController = new PIDController(0.01, 0, 0);
      public static final ModuleConstants constants =
          new ModuleConstants(angleMotorID, driveMotorID, canCoderID, angleOffset.getDegrees(), angleController, false, false, false);
    }
    
}   