// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveConstants;
public class TeleopDrive extends Command {
  private Swerve drive;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier theta;
  private BooleanSupplier field;

  /**
   * This method will take in the raw inputs of the xbox controller and process and pass them into the drive method of the Swerve subsystem 
   * Then the needed velocity and posistion for the drive and azimuth respecitvely are calcauted and applied
   * @param drive Swerve subsystem that needs to be passed in
   * @param x Xbox controller 'x' demand or Left Y joystickto move back and forth
   * @param y Xbox controller 'y' demand or Left X joystick to move right and left
   * @param theta Xbox controller 'theta' demand or Right X joystick for rotation
   * @param field Xbox controller input of a boolean determining if the robot should be field or robot oriented
   */
  public TeleopDrive(Swerve drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier field) {
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.field = field;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVal = x.getAsDouble();
    double yVal = y.getAsDouble();
    double thetaVal = theta.getAsDouble();

    drive.drive(
      // The value as of now is between 0 and 1 // 
      // Multiplying by the max speed helps finding the real demand that was applied //
      new Translation2d(xVal, yVal).times(SwerveConstants.maxLinearSpeed), 
      thetaVal, 
      !field.getAsBoolean()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
