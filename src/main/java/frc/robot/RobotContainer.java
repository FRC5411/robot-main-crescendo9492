// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {

  private Intake intake;
  private Shooter shooter;

  private CommandXboxController operator;

  public RobotContainer() {
    intake = new Intake();
    shooter = new Shooter(intake);

    operator = new CommandXboxController(Constants.k_operatorID);

    configureBindings();
  }

  private void configureBindings() {
    // right bumper intakes note
    operator.rightBumper()
      .whileTrue(new InstantCommand(() -> intake.intake()))
      .onFalse(intake.retract());

    // 'a' button outtakes note
    operator.a()
      .whileTrue(new InstantCommand(() -> intake.outtake()))
      .onFalse(new InstantCommand(() -> intake.zero()));

    // Left bumper shoots Speaker
    operator.leftBumper()
      .whileTrue(new InstantCommand(() -> shooter.shootSpeaker()))
      .onFalse(new InstantCommand(() -> shooter.zero()));

    // Left joystick click/roll shoots amp, binding to be added at a later date
    operator.leftStick()
      .whileTrue(new InstantCommand(() -> shooter.shootAmp()))
      .onFalse(new InstantCommand(() -> shooter.zero()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
