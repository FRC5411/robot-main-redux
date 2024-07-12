// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;
/** 
 * This is a record, which functions as a immutable data class to store values
 * Values can be called from ModuleConstants using '.constant()'
 */
public record ModuleConstants(
    int azimuthID,
    int driveID,
    int encoderID,
    double offset,
    boolean azimuthFlipped,
    boolean driveFlipped,
    boolean absoluteFlipped    
) {}
