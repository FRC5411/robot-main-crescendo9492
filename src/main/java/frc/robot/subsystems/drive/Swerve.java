// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.File;
import java.util.function.DoubleSupplier;

import javax.management.RuntimeErrorException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
  
  private SwerveDrive swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(DriveConstants.k_rateLimit);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(DriveConstants.k_rateLimit);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.k_rateLimitRotation);

  public Swerve() {

    /* Creates a SwerveDrive object with a set max speed by parsing a directory of module values and other properties into a SwerveDrive object. 
    If unsuccessful, a Runtime error is thrown. */
    try {
      swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(0);
    }
    catch (Exception e){
      throw new RuntimeErrorException(null);
    }
  }

  /* The typical drive command that is normally in the commands folder, it has simply been moved into the subsystem folder. */
  public Command drive(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
      double translation = translationLimiter.calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), DriveConstants.k_deadband));
      double strafe = strafeLimiter.calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), DriveConstants.k_deadband));
      double rotation = rotationLimiter.calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), DriveConstants.k_deadband));

      drive(
        new Translation2d(translation, strafe), 
        rotation, 
        true, 
        false
      );
    }).withName("TeleopSwerve");
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerve.drive(
      translation, 
      rotation, 
      fieldRelative, 
      isOpenLoop);
  }

  public void setBrakeIdle(boolean isBrake) {
    swerve.setMotorIdleMode(isBrake);
  }

  @Override
  public void periodic() {
    swerve.updateOdometry();
  }

}
