// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
