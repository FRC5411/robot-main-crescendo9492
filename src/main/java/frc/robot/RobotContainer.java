// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommand;

public class RobotContainer {

  private Arm arm;

  public CommandXboxController operator;

  public RobotContainer() {

    operator = new CommandXboxController(Constants.k_operatorID);

    arm = new Arm(operator.getHID());

    // Manual:
    arm.setDefaultCommand(new ArmCommand(
      () -> operator.getRightY(),
      arm
    ));
    
    configureBindings();
  }

  private void configureBindings() {
    operator.x().onTrue(new InstantCommand(() -> arm.goToIntakePos()));
    operator.y().onTrue(new InstantCommand(() -> arm.goToShootPos()));
    operator.a().onTrue(new InstantCommand(() -> arm.goToAmpPos()));
  }

  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
