// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {
  
  private Swerve swerve;
  private CommandXboxController driveController;

  public RobotContainer() {
    swerve = new Swerve();
    driveController = new CommandXboxController(Constants.driverPort);

    swerve.setDefaultCommand(
      new DriveCommand(
        swerve, 
        () -> -driveController.getLeftY(), 
        () -> driveController.getLeftX(), 
        () -> driveController.getRightX(), 
        true
      )
    );
  }




  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
