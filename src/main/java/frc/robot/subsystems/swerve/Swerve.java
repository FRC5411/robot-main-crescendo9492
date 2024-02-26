package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

  private SwerveModule[] swerveModules;
  private SwerveModulePosition[] swerveModulePoses;
  private SwerveDriveOdometry swerveOdometry;
  private Field2d field;

  private Pigeon2 gyro;

  private double t;
  private double r;

  public Swerve() {
    swerveModules = new SwerveModule[] {
        new SwerveModule(SwerveConstants.k_FLModuleID),
        new SwerveModule(SwerveConstants.k_FRModuleID),
        new SwerveModule(SwerveConstants.k_BLModuleID),
        new SwerveModule(SwerveConstants.k_BRModuleID)
    };

    swerveModulePoses = new SwerveModulePosition[] {
        new SwerveModulePosition(
            swerveModules[SwerveConstants.k_FLModuleID].getDriveEncoderPosition(),
            swerveModules[SwerveConstants.k_FLModuleID].getAngle()),
        new SwerveModulePosition(
            swerveModules[SwerveConstants.k_FRModuleID].getDriveEncoderPosition(),
            swerveModules[SwerveConstants.k_FRModuleID].getAngle()),
        new SwerveModulePosition(
            swerveModules[SwerveConstants.k_BLModuleID].getDriveEncoderPosition(),
            swerveModules[SwerveConstants.k_BLModuleID].getAngle()),
        new SwerveModulePosition(
            swerveModules[SwerveConstants.k_BRModuleID].getDriveEncoderPosition(),
            swerveModules[SwerveConstants.k_BRModuleID].getAngle())
    };

    gyro = new Pigeon2(SwerveConstants.k_pigeonID);
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), swerveModulePoses);

    field = new Field2d();
  }

  public void swerveDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    // Calculate speeds based on joystick input (Either field oriented or robot
    // oriented)
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));

    // Optimize Speed and Angles
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleID], isOpenLoop);
    }

    SmartDashboard.putBoolean("Field Oriented", fieldRelative);
  }

  // Optimize Speed and Angles only
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleID], false);
    }
  }

  // For autonomous
  public void driveFromChassisSpeeds(ChassisSpeeds wheelSpeeds) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(wheelSpeeds);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleID], false);
    }
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), swerveModulePoses, pose);
  }

  public void resetModules() {
    for (SwerveModule mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public void zeroGyro() {
    gyro.setYaw(0.0);
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees((360 - gyro.getYaw().getValue()) % 360)
        : Rotation2d.fromDegrees(gyro.getYaw().getValue() % 360);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule mod : swerveModules) {
      states[mod.moduleID] = mod.getState();
    }

    return states;
  }

  public SwerveModulePosition[] getPositions() {
    for (SwerveModule mod : swerveModules) {
      swerveModulePoses[mod.moduleID] = new SwerveModulePosition(
          mod.getDriveEncoderPosition(),
          mod.getAngle());
    }

    return swerveModulePoses;
  }

  public void odom(DoubleSupplier trans, DoubleSupplier rot) {
    t = trans.getAsDouble();
    r = rot.getAsDouble();
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putData(field);

    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber("Module " + mod.moduleID + " Cancoder ",
          mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Module " + mod.moduleID + " Cancoder Offset ",
          mod.getCanCoderOffset().getDegrees());
      SmartDashboard.putNumber("Module " + mod.moduleID + " Integrated ",
          mod.getState().angle.getRotations());
      SmartDashboard.putNumber("Module " + mod.moduleID + " Velocity ",
          mod.getState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("transSup", t);
    SmartDashboard.putNumber("rotSup", r);
  }
}