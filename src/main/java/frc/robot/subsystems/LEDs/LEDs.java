// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDs extends SubsystemBase {

  public AddressableLED LEDs;
  public AddressableLEDBuffer LEDBuffer;

  

  public LEDs() {
    LEDs = new AddressableLED(LEDConstants.k_LEDPort);
    
    LEDBuffer = new AddressableLEDBuffer(LEDConstants.k_length);
    LEDs.setLength(LEDBuffer.getLength());

    LEDs.setData(LEDBuffer);
    LEDs.start();

    
  }
  
  // Purple will be the default color of the LEDs (set to Eclipse)
  public void setLEDsPurple() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 174, 55, 255);
    }

    LEDs.setData(LEDBuffer);
    // LEDConstants.setPurple = true;

  }

  public void setLEDsOrange() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 255, 94, 5);
    }

    LEDs.setData(LEDBuffer);
    LEDConstants.k_isOrange = true;

  }

  public void setLEDsBlue() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 85, 206, 255);
    }

    LEDs.setData(LEDBuffer);
    LEDConstants.k_isBlue = true;

  }

  public void blinkLEDsOrange() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 255, 94, 5);
    }
  }

  public Command blinkLEDs() {
    return new SequentialCommandGroup (
      new InstantCommand(() -> blinkLEDsOrange()),
      new WaitCommand(LEDConstants.k_waitTime),
      new InstantCommand(() -> setLEDsPurple())
    );
  }
  
  public Command toggleOrange() {
    LEDConstants.k_isBlue = false;
    if (LEDConstants.k_isOrange) {
      LEDConstants.k_isOrange = false;
      return new InstantCommand(() -> setLEDsPurple());
    }

    LEDConstants.k_isOrange = true;
    return new InstantCommand(() -> setLEDsOrange());
  }

  public Command toggleBlue() {
    LEDConstants.k_isOrange = false;
    if (LEDConstants.k_isBlue) {
      LEDConstants.k_isBlue = false;
      return new InstantCommand(() -> setLEDsPurple());
    }

    LEDConstants.k_isBlue = true;
    return new InstantCommand(() -> setLEDsBlue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
