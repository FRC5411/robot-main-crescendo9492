// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {

  private Intake intake;

  private CommandXboxController operator;

  public RobotContainer() {
    intake = new Intake();

    operator = new CommandXboxController(Constants.k_operatorID);

    configureBindings();
  }

  private void configureBindings() {
    operator.rightBumper()
      .whileTrue(new InstantCommand(() -> intake.intake()))
      .onFalse(new InstantCommand(() -> intake.zero()));

    operator.leftBumper()
      .whileTrue(new InstantCommand(() -> intake.outake()))
      .onFalse(new InstantCommand(() -> intake.zero()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
