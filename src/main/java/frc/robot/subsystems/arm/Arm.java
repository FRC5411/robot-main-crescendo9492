// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Climb. */

  private CANSparkMax armMotor;

  private RelativeEncoder armEncoder;
  private ProfiledPIDController armController;
  private ArmFeedforward feedForward;

  public Arm(XboxController controller) {

    armMotor = new CANSparkMax(ArmConstants.k_armMotorID, MotorType.kBrushless);
    
    armEncoder = armMotor.getEncoder();
    armController = ArmConstants.k_armPID;
    feedForward = ArmConstants.k_armFeedforward;

    configure();
  }

  // Moves arm down to pick up note
  public void goToIntakePos() {
    setSpeed(setPID(ArmConstants.k_intakeSetpoint) + setfeedForward()); 
  }

  // Moves arm up to shoot note
  public void goToShootPos() {
    setSpeed(setPID(ArmConstants.k_shootSetpoint) + setfeedForward());
  }

  // Depending on the value of the joysticks, move the arm up or down
  public void runArm(double voltage) {

    if (voltage > 0) setSpeed(ArmConstants.k_armSpeed + setfeedForward());
    else if (voltage == 0) setSpeed(setfeedForward());
    else setSpeed(-ArmConstants.k_armSpeed + setfeedForward());
  }

  // Calculate the motor speed based on PIDs
  public double setPID(double setpoint) {
    return armController.calculate(getPosition().getRotations(), setpoint);
  }

  // Calculate the speed to move the arm up and down at the same speed (by taking into account gravity)
  public double setfeedForward() {
    return feedForward.calculate(getPosition().getRadians(), ArmConstants.k_armSpeed);
  }

  // Sets the speed of the motors if it is within the bounds of the robot
  public void setSpeed(double armSpeed) {
    if (isInBound(getPosition(), armSpeed)) {
      armMotor.set(armSpeed);
    }
    else {
      armMotor.set(ArmConstants.k_speedZero);
    }
  }

  // Checks if the arm is within its upper and lower bounds
  public boolean isInBound(Rotation2d setpoint, double armSpeed) {

    if (setpoint.getRotations() > ArmConstants.k_upperBound && armSpeed > 0.0) return false;
    else if (setpoint.getRotations() < ArmConstants.k_lowerBound && armSpeed < 0.0) return false;
    return true;
  }

  // Get the position of the encdoer, taking into account offsets
  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(armEncoder.getPosition() + ArmConstants.k_armEncoderOffset.getRotations());
  }

  // configure the arm motor, encoder, and PID controller
  public void configure() {
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(ArmConstants.k_smartCurrentLimit);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)ArmConstants.k_upperBound);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)ArmConstants.k_lowerBound);
    
    armEncoder.setPositionConversionFactor(ArmConstants.k_positionConversionFactor);
    armEncoder.setVelocityConversionFactor(ArmConstants.k_velocityConversionFactor);

    armController.setP(ArmConstants.k_armP);
    armController.setI(ArmConstants.k_armI);
    armController.setD(ArmConstants.k_armD);
    armController.setTolerance(ArmConstants.k_tolerance);

    armMotor.burnFlash();
    armMotor.clearFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Val", getPosition().getRotations());
    SmartDashboard.putNumber("Arm Speed", armMotor.get());
    SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
  }
}
