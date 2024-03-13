// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Sensor extends SubsystemBase {
  /** Creates a new Sensor. */
  private DigitalInput sensor;
  public Sensor() {
    sensor = new DigitalInput(LightConstants.k_DIOPort);
  }

  public boolean getNoteDetected() {
    if (sensor.get()) {
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
