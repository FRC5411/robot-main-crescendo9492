// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.lights.Light;

public class RobotContainer {
public CommandXboxController driverController;
public Light lights;

  public RobotContainer() {

    lights = new Light();

    driverController = new CommandXboxController(Constants.k_operatorID);

    lights.setLEDsPurple();

    configureBindings();
  }

  private void configureBindings() {
    // Button bindings:
    // left bumper = Coopertition
    // right bumper = Power up

    driverController.leftBumper().onTrue(
      lights.toggleOrange()
    );

    driverController.rightBumper().onTrue(
      lights.toggleBlue()
    );
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
