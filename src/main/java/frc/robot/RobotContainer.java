// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDs.LEDConstants;
import frc.robot.subsystems.LEDs.LEDs;

public class RobotContainer {
public CommandXboxController driverController;
public LEDs LEDs;

  public RobotContainer() {
    driverController = new CommandXboxController(Constants.operatorPort);
    configureBindings();
  }

  private void configureBindings() {
    // Button bindings:
    // left bumper = Coopertition
    // right bumper = Power up

    driverController.leftBumper().onTrue(
      LEDs.toggleOrange()
    );

    driverController.rightBumper().onTrue(
      LEDs.toggleBlue()
    );
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
