// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Swerve;

public class DriveCommand extends Command {

    private Swerve robotSwerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private Boolean robotCentricSup;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter rotationLimiter;
    
    public DriveCommand(Swerve robotSwerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {
        
        this.robotSwerve = robotSwerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        this.robotCentricSup = robotCentricSup;

        translationLimiter = new SlewRateLimiter(3.0);
        strafeLimiter = new SlewRateLimiter(3.0);
        rotationLimiter = new SlewRateLimiter(8.0);

        addRequirements(robotSwerve);
    }

    @Override
    public void execute() {
        double translationVal = translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), DriveConstants.stickDeadband)
        );
        double strafeVal = strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), DriveConstants.stickDeadband)
        );
        double rotationVal = rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), DriveConstants.stickDeadband)
        );

        robotSwerve.swerveDrive(
            new Translation2d(translationVal, strafeVal).times(DriveConstants.maxSpeed), 
            rotationVal * DriveConstants.maxAngularVelocity, 
            !robotCentricSup, 
            true);
        
        robotSwerve.odom(translationSup, rotationSup);
    } 
}
