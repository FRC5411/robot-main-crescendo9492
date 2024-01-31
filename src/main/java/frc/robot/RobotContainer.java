// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {
  
  private Swerve swerve;
  private XboxController driveController;

  public RobotContainer() {
    swerve = new Swerve();
    driveController = new XboxController(Constants.k_driverControllerID);

    swerve.setDefaultCommand(
    new RunCommand(() -> swerve.drive(
        () -> MathUtil.applyDeadband(driveController.getLeftY(), 0.2), 
        () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.2), 
        () -> MathUtil.applyDeadband(driveController.getRightX(), 0.2),
        true,
        false
    ), swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
