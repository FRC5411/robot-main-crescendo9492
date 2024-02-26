package frc.robot.subsystems.LEDs;

public class LEDConstants {
    
  // LED port - Must be a PWM header, not MXP or DIO - TO BE CONFIGURED:
  public static final int k_LEDPort = 1;

  // Length of LEDs:
  public static final int k_length = 15;

  // Wait Command length to set time for LEDs to blink:
  public static final double k_waitTime = 0.5;

  // Booleans of whether or not LEDs are orange, blue, or purple:
  public static boolean k_isOrange = false;
  public static boolean k_isBlue = false;
}