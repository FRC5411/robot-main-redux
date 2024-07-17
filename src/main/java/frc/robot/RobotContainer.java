// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.drive.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CommandXboxController pilot = new CommandXboxController(0);
  private final Swerve swerve = new Swerve();

  public RobotContainer() {
    swerve.setDefaultCommand(
      new TeleopDrive(
        // Put negative sings inside of the method in the lambda to flip directions for robot movement //
        swerve, 
        () -> applyDeadband(pilot.getLeftY()), 
        () -> applyDeadband(pilot.getLeftX()), 
        () -> applyDeadband(pilot.getRightX()), 
        () -> true));
    configureBindings();
  }

  // Fix the deadband depending on how bad the the controller is // 
  private double applyDeadband(double demand){
    if(-0.2 < demand && demand < 0.2){
      return 0;
    }

    return demand;
  }

  private void configureBindings() {
    // Find the minimum voltage needed to move robot // 
    // Which will help find kS //
    pilot.a().onTrue(new InstantCommand(() -> {resetModules();}));

    pilot.x().onTrue(new InstantCommand(() -> swerve.zeroHeading()));

  }

  public void resetModules(){
    swerve.FR.resetAzimuthPosistion();
    swerve.FL.resetAzimuthPosistion();
    swerve.BR.resetAzimuthPosistion();
    swerve.BL.resetAzimuthPosistion();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
