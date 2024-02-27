// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Climb. */

  private CANSparkMax armMotor;

  private RelativeEncoder armEncoder;
  private SparkPIDController armController;


  private XboxController controller;

  public Arm(XboxController controller) {

    this.controller = controller;

    armMotor = new CANSparkMax(ArmConstants.k_armMotorID, MotorType.kBrushless);
    

    armEncoder = armMotor.getEncoder();
    armController = armMotor.getPIDController();

    configure();
  }

  public void runArm(double voltage) {

    if (voltage > 0) setSpeed(ArmConstants.k_armSpeed + feedForward());
    else if (voltage == 0) setSpeed(feedForward());
    else setSpeed(-ArmConstants.k_armSpeed + feedForward());
  }

  public void goToIntakePos() {
    PID(ArmConstants.k_intakeSetpoint);
    feedForward();
  }

  public void goToShootPos() {
    PID(ArmConstants.k_shootSetpoint);
    feedForward();
  }

  public double PID(double setpoint) {
    return ArmConstants.k_armPID.calculate(getPosition().getRotations(), setpoint);
  }

  public double feedForward() {
    return ArmConstants.k_armFeedforward.calculate(getPosition().getRadians(), ArmConstants.k_armSpeed);
  }

  public void setSpeed(double armSpeed) {
    if (isInBound(getPosition(), armSpeed)) {
      armMotor.set(armSpeed);
      // armController.setReference(getPosition(), ControlType.kPosition, 0)
    }
    else {
      controller.setRumble(RumbleType.kBothRumble, Constants.k_rumbleStrength);
      armMotor.set(feedForward());
    }
  }

  public boolean isInBound(Rotation2d setpoint, double armSpeed) {

    if (setpoint.getRotations() > ArmConstants.k_upperBound && armSpeed > 0.0) return false;
    else if (setpoint.getRotations() < ArmConstants.k_lowerBound && armSpeed < 0.0) return false;
    return true;
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(armEncoder.getPosition() + ArmConstants.k_armEncoderOffset.getRotations());
  }

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
    armController.setFeedbackDevice(armEncoder);

    armMotor.burnFlash();
    armMotor.clearFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Value", getPosition().getRotations());
    SmartDashboard.putNumber("Arm Speed", armMotor.get());
    SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
  }
}
