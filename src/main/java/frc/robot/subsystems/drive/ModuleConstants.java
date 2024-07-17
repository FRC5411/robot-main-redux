// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * This is a record, which functions as a immutable data class to store values
 * Values can be called from ModuleConstants using '.constant()'
 * @param azimuthID port of the azimuth motor
 * @param driveID port of the drive motor
 * @param encoderID port of the CANcoder
 * @param offset offset of the CANcoder in rotations
 * @param azimuthFlipped is the azimuth flipped?
 * @param driveFlipped is the drive flipped?
 * @param absoluteFlipped is the CANCoder flipped?
 */
public record ModuleConstants(
    int azimuthID,
    int driveID,
    int encoderID,
    Rotation2d offset,
    boolean azimuthFlipped,
    boolean driveFlipped,
    boolean absoluteFlipped    
) {}
