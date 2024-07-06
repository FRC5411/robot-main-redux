// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
public class TeleopDrive extends Command {
  private Swerve drive;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier theta;
  private BooleanSupplier field;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter thetaLimiter = new SlewRateLimiter(3.0);

  /** Creates a new TeleopDrive. */
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
    double xVal = xLimiter.calculate(x.getAsDouble());
    double yVal = yLimiter.calculate(y.getAsDouble());
    double thetaVal = thetaLimiter.calculate(theta.getAsDouble()); 

    drive.drive(
      new Translation2d(xVal, yVal), 
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
