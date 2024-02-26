// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Light extends SubsystemBase {

  public AddressableLED LEDs;
  public AddressableLEDBuffer LEDBuffer;

  

  public Light() {
    LEDs = new AddressableLED(LightConstants.k_LEDPort);
    
    LEDBuffer = new AddressableLEDBuffer(LightConstants.k_length);
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
    LightConstants.k_isOrange = false;
    LightConstants.k_isBlue = false;
  }

  public void setLEDsOrange() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 255, 94, 5);
    }

    LEDs.setData(LEDBuffer);
    LightConstants.k_isOrange = true;
    LightConstants.k_isBlue = false;
  }

  public void setLEDsBlue() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 85, 206, 255);
    }

    LEDs.setData(LEDBuffer);
    LightConstants.k_isBlue = true;
    LightConstants.k_isOrange = false;
  }

  public void blinkLEDsOrange() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 255, 94, 5);
    }
  }

  public Command blinkLEDs() {
    return new SequentialCommandGroup (
      new InstantCommand(() -> blinkLEDsOrange()),
      new WaitCommand(LightConstants.k_waitTime),
      new InstantCommand(() -> setLEDsPurple())
    );
  }
  
  public Command toggleOrange() {
    if (LightConstants.k_isOrange) {
      return new InstantCommand(() -> setLEDsPurple());
    }

    return new InstantCommand(() -> setLEDsOrange());
  }

  public Command toggleBlue() {
    if (LightConstants.k_isBlue) {
      return new InstantCommand(() -> setLEDsPurple());
    }

    return new InstantCommand(() -> setLEDsBlue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
