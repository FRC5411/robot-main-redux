// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class ModuleConstants {
    public final int azimuthID; 
    public final int driveID;
    public final int encoderID;
    public final double offset;
    public final PIDController angleController;
    public final boolean azimuthFlipped;
    public final boolean driveFlipped;
    public final boolean absoulteFlipped;

    public ModuleConstants(
        int azimuthID, 
        int driveID, 
        int encoderID, 
        double offset, 
        PIDController angleController,
        boolean azimuthFlipped, 
        boolean driveFlipped,
        boolean absoluteFlipped){

        this.azimuthID = azimuthID;
        this.driveID = driveID;
        this.encoderID = encoderID;
        this.offset = offset;
        this.angleController = angleController;
        this.azimuthFlipped = azimuthFlipped;
        this.driveFlipped = driveFlipped;
        this.absoulteFlipped = absoluteFlipped;
    }
}
