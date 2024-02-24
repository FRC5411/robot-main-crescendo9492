// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
public class Shooter extends SubsystemBase {
;
  /** Creates a new Shooter. */
  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;
  private CANSparkMax indexerMotor;
  private CANSparkMax pivotMotor;
  public Shooter() {
    leftShooterMotor = new CANSparkMax(ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ShooterConstants.rightShooterMotorID, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(ShooterConstants.indexerMotorID, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ShooterConstants.pivotMotorID, MotorType.kBrushless);
  }
  public Rotation2d getPosition() {
    return new Rotation2d(Math.toRadians(pivotMotor.getEncoder().getPosition() * 360) / ShooterConstants.k_gearRatio);
  }
  public Rotation2d getVelocity() {
    return new Rotation2d(Units.rotationsPerMinuteToRadiansPerSecond(pivotMotor.getEncoder().getVelocity()));
  }
  public void setMotorVolts(double voltage) {
    if (isInBounds(voltage)) {
      pivotMotor.setVoltage(voltage);
    }
    else {
      pivotMotor.setVoltage(0.1 * Math.signum(voltage));
      System.out.println("Soft limits in effect!!");
    }
  }
  public boolean isInBounds(double voltage) {
    if (getPosition().getDegrees() < ShooterConstants.k_upperBound && getPosition().getDegrees() > ShooterConstants.k_lowerBound) {
      return true;
    }
    else {
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Encoder: ", getPosition().getDegrees());
  }
}