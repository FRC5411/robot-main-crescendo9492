// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private DigitalInput indexerSensor;
 
  public Intake() {
    indexerSensor = new DigitalInput(IntakeConstants.k_sensorID);
  }

  // Detects whether note is in intake, will be connected to motors at a later time
  public boolean getNoteDetected() {
    if (indexerSensor.get()) {
      System.out.println("Note detected! ");
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
