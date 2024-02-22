package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  public int moduleID;

  private CANSparkMax driveMotor;
  private CANSparkMax azimuthMotor;
  private CANcoder angleEncoder;
  private Rotation2d angleOffset;

  private Rotation2d lastAngle;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder azimuthEncoder;

  private SparkPIDController driveController;
  private SparkPIDController azimuthController;

  private SimpleMotorFeedforward feedForward;

  public SwerveModule(int moduleID) {

    this.moduleID = moduleID;

    switch(moduleID){
      // FL Module
      case 0:
        driveMotor = new CANSparkMax(11, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(21, MotorType.kBrushless);

        angleEncoder = new CANcoder(31);
        angleOffset = Rotation2d.fromRotations(0.261230);
        break;

      // FR Module
      case 1:
        driveMotor = new CANSparkMax(12, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(22, MotorType.kBrushless);

        angleEncoder = new CANcoder(32);
        angleOffset = Rotation2d.fromRotations(-0.090576);
        break;

      // BL Module
      case 2:
        driveMotor = new CANSparkMax(13, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(23, MotorType.kBrushless);

        angleEncoder = new CANcoder(33);
        angleOffset = Rotation2d.fromRotations(0.059082);
        break;

      // BR Module
      case 3:
        driveMotor = new CANSparkMax(14, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(24, MotorType.kBrushless);

        angleEncoder = new CANcoder(34);
        angleOffset = Rotation2d.fromRotations(0.075928);
        break;

      default:
        throw new RuntimeException("Module ID not found");
    }

    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();

    azimuthEncoder = azimuthMotor.getEncoder();
    azimuthController = azimuthMotor.getPIDController();

    configureDriveMotor();
    configureAzimuthMotor();

    feedForward = new SimpleMotorFeedforward(DriveConstants.driveKS, DriveConstants.driveKV, DriveConstants.driveKA);
    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    setSpeed(desiredState, isOpenLoop);
    setAngle(desiredState);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.maxSpeed;

        driveMotor.set(percentOutput);
    }
    else {
        driveController.setReference(
            desiredState.speedMetersPerSecond, 
            ControlType.kVelocity, 
            0, 
            feedForward.calculate(desiredState.speedMetersPerSecond)
        );
    }
}

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = 
        (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxSpeed * 0.01)) // Prevent jittering
            ? lastAngle 
            : desiredState.angle;
    
    azimuthController.setReference(angle.getRotations(), ControlType.kPosition);
    lastAngle = angle;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(azimuthEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public Rotation2d getCanCoderOffset() {
    return Rotation2d.fromRotations(zeroTo360Scope(getCanCoder()));
}

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public double getDriveEncoderPosition() {
    return driveEncoder.getPosition();
  }

  public void resetToAbsolute() {
    double absolutePosition = zeroTo360Scope(getCanCoder());

    REVLibError a = azimuthEncoder.setPosition(absolutePosition);

    System.out.println(a);
}

public double zeroTo360Scope(Rotation2d rotations) {
  rotations.plus(angleOffset);
  if (rotations.getRotations() < 0) rotations.plus(Rotation2d.fromRotations(1));
  return rotations.getRotations();
}
  
public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
  double targetAngle = placeInAppropriate0To360Scope(
    currentAngle.getDegrees(), 
    desiredState.angle.getDegrees());
  double targetSpeed = desiredState.speedMetersPerSecond;
  double delta = targetAngle - currentAngle.getDegrees();

  if (Math.abs(delta) > 90) {
    targetSpeed = -targetSpeed;
    targetAngle = delta > 90 ? 
      (targetAngle -= 180) 
      : (targetAngle += 180);
  }

  return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
}

private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
  double lowerBound;
  double upperBound;
  double lowerOffset = scopeReference % 360;

  if (lowerOffset >= 0) {
    lowerBound = scopeReference - lowerOffset;
    upperBound = scopeReference + (360 - lowerOffset);
  } 
  else {
    upperBound = scopeReference - lowerOffset;
    lowerBound = scopeReference - (360 + lowerOffset);
  }
  while (newAngle < lowerBound) {
    newAngle += 360;
  }
  while (newAngle > upperBound) {
    newAngle -= 360;
  }
  if (newAngle - scopeReference > 180) {
    newAngle -= 360;
  } 
  else if (newAngle - scopeReference < -180) {
    newAngle += 360;
  }

  return newAngle;
}
  
private void configureAzimuthMotor() {
    azimuthMotor.restoreFactoryDefaults();

    azimuthMotor.setSmartCurrentLimit(DriveConstants.azimuthContinuousCurrentLimit);
    azimuthMotor.enableVoltageCompensation(DriveConstants.voltageComp);

    azimuthMotor.setInverted(DriveConstants.azimuthInvert);
    azimuthMotor.setIdleMode(DriveConstants.azimuthNeutralMode);

    azimuthEncoder.setPosition(0.0);
    azimuthEncoder.setPositionConversionFactor(DriveConstants.azimuthConversionFactor);

    azimuthController.setP(DriveConstants.angleKP);
    azimuthController.setI(DriveConstants.angleKI);
    azimuthController.setD(DriveConstants.angleKD);
    azimuthController.setFF(DriveConstants.angleKFF);

    azimuthController.setFeedbackDevice(azimuthEncoder);

    azimuthMotor.burnFlash();

    resetToAbsolute(); 
  }
  
private void configureDriveMotor() {
    driveMotor.restoreFactoryDefaults();

    driveMotor.setSmartCurrentLimit(DriveConstants.driveContinuousCurrentLimit);
    driveMotor.enableVoltageCompensation(DriveConstants.voltageComp);

    driveMotor.setInverted(DriveConstants.driveInvert);
    driveMotor.setIdleMode(DriveConstants.driveNeutralMode);

    driveEncoder.setPosition(0.0);
    driveEncoder.setPositionConversionFactor(DriveConstants.driveConversionPositionFactor);
    driveEncoder.setVelocityConversionFactor(DriveConstants.driveConversionVelocityFactor);

    driveController.setP(0.1);
    driveController.setI(0.0);
    driveController.setD(0.0);
    driveController.setFF(0.0);

    driveController.setFeedbackDevice(driveEncoder);

    driveMotor.burnFlash();
  }
}