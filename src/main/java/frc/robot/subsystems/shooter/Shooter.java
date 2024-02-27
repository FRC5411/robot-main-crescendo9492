// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax leftShootMotor;
  private CANSparkMax rightShootMotor;

  public Shooter() {
    leftShootMotor = new CANSparkMax(ShooterConstants.k_leftShootMotorID, MotorType.kBrushless);
    rightShootMotor = new CANSparkMax(ShooterConstants.k_rightShootMotorID, MotorType.kBrushless);

    rightShootMotor.follow(leftShootMotor, !ShooterConstants.k_isInverted);
    leftShootMotor.setInverted(ShooterConstants.k_isInverted);
  }

  // Set motor speeds to the speed needed to score Speaker
  public void shootSpeaker() {
    leftShootMotor.set(ShooterConstants.k_shootSpeaker);
  }

  // Set motor speeds to the speed needed to score Amp, if different from Speaker
  public void shootAmp() {
    leftShootMotor.set(ShooterConstants.k_shootAmp);
  }

  // Set motor speeds to zero
  public void zero() {
    leftShootMotor.set(ShooterConstants.k_shootZero);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Configure motors
  public void configure(CANSparkMax motor) {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(ShooterConstants.k_smartCurrentLimit);
    motor.burnFlash();
    motor.clearFaults();
  }
}
