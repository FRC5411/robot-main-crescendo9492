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
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      //LEDBuffer.setRGB(i, 174, 55, 255);
      LEDBuffer.setRGB(i, 95, 0, 160);
    }

    LEDs.setData(LEDBuffer);
    LightConstants.k_isOrange = false;
    LightConstants.k_isBlue = false;
  }

  // Solid Orange will be the signal for coopertition bonus (set to Flame)
  public void setLEDsOrange() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      //LEDBuffer.setRGB(i, 255, 94, 5);
      LEDBuffer.setRGB(i, 250, 41, 0);
    }

    LEDs.setData(LEDBuffer);
    LightConstants.k_isOrange = true;
    LightConstants.k_isBlue = false;
  }

  // Solid Blue will be the signal for amplification bonus (set to Ocean)
  public void setLEDsBlue() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      // LEDBuffer.setRGB(i, 85, 206, 255);
      LEDBuffer.setRGB(i, 0, 171, 240);
    }

    LEDs.setData(LEDBuffer);
    LightConstants.k_isOrange = false;
    LightConstants.k_isBlue = true;
  }


  
  // Blinking Orange twice will be the signal for note detection when inside the indexer
  public Command blinkLEDsOrange() {
    return new SequentialCommandGroup (
      new InstantCommand(() -> setLEDsOrange()),
      new WaitCommand(LightConstants.k_waitTime),
      new InstantCommand(() -> setLEDsPurple()),
      new WaitCommand(LightConstants.k_waitTime),
      new InstantCommand(() -> setLEDsOrange()),
      new WaitCommand(LightConstants.k_waitTime),
      new InstantCommand(() -> setLEDsPurple())
    );
  }
  
  // Toggle between Orange and Purple to signal for coopertition bonus
  public Command toggleOrange() {
    if (LightConstants.k_isOrange) {
      return new InstantCommand(() -> setLEDsPurple());
      }
    

    LightConstants.k_isOrange = true;
    return new InstantCommand(() -> setLEDsOrange()); 
  
  }

  // Toggle between Blue and Purple to signal for coopertition bonus
  public Command toggleBlue() {
    if (LightConstants.k_isBlue) {
      return new InstantCommand(() -> setLEDsPurple());
    }

    // LightConstants.k_isBlue = true;
    return new InstantCommand(() -> setLEDsBlue());
  }

  

  private int m_rainbowFirstPixelHue = 1;
  public void setRainbow() {
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i*180 / LEDBuffer.getLength())) % 180;
      LEDBuffer.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 3;

    m_rainbowFirstPixelHue %= 180;

    LEDs.setData(LEDBuffer);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
