package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommand;

public class RobotContainer {
  private Swerve swerve;
  private CommandXboxController driveController;

  public RobotContainer() {
    swerve = new Swerve();
    driveController = new CommandXboxController(Constants.driverPort);

    swerve.setDefaultCommand(
        new SwerveCommand(
            swerve,
            () -> -driveController.getLeftY(),
            () -> driveController.getLeftX(),
            () -> driveController.getRightX(),
            true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
