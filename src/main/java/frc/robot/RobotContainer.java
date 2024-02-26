// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.ShooterConstants;
//import frc.robot.subsystems.shooter.;


public class RobotContainer {
  
  public CommandXboxController operatorController;
  public ShooterCommand shooterCommand = new ShooterCommand();


  public RobotContainer() {
    operatorController = new CommandXboxController(ShooterConstants.k_operatorPort);

    configureBindings();
  }

  public void configureBindings() {
    operatorController.a().whileTrue(new InstantCommand(() -> shooterCommand.startIndexerMotor()));
    System.out.println("Button being pressed");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  }


  


