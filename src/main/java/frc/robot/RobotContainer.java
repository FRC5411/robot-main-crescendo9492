// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {
  
  private Swerve swerveDrive;
  private CommandXboxController driveController;

  public RobotContainer() {
    swerveDrive = new Swerve();
    driveController = new CommandXboxController(Constants.k_driverControllerID);
  }

  public void configureSwerve() {
    swerveDrive.setDefaultCommand(
      swerveDrive.drive(
        () -> driveController.getRawAxis(XboxController.Axis.kLeftY.value), 
        () -> driveController.getRawAxis(XboxController.Axis.kLeftX.value), 
        () -> driveController.getRawAxis(XboxController.Axis.kRightX.value)
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setBrakeIdle(boolean isBrake) {
    swerveDrive.setBrakeIdle(true);
  }
}
