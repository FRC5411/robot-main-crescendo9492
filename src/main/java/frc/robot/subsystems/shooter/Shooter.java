// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;

  private boolean m_launcherRunning;

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftCanId, MotorType.kBrushless);
    leftShooterMotor.setInverted(false);
    leftShooterMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    leftShooterMotor.setIdleMode(IdleMode.kBrake);

    leftShooterMotor.burnFlash();

    rightShooterMotor =
        new CANSparkMax(ShooterConstants.kRightCanId, MotorType.kBrushless);
    rightShooterMotor.setInverted(false);
    rightShooterMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    rightShooterMotor.setIdleMode(IdleMode.kBrake);

    rightShooterMotor.burnFlash();

    m_launcherRunning = false;
    
  }

  public void runlauncher() {
    m_launcherRunning = true;
  }

  public void stopLauncher() {
    m_launcherRunning = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_launcherRunning) {
      leftShooterMotor.set(ShooterConstants.kLeftPower);
      rightShooterMotor.set(ShooterConstants.KRightPower);
    }
    else {
      leftShooterMotor.set(0.0);
      rightShooterMotor.set(0.0);
    }
  }
}
