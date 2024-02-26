package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public final class VisionConstants {

  // FIND MEASUREMENTS ONCE CAMERA IS MOUNTED:
  public static final double k_cameraHeightMeters = 0;
  public static final double k_targetHeightMeters = 0;
  public static final double k_cameraPitchRadians = 0;

  // This measurement is if we have the fixed shooter height be made specifically
  // for shooting into the speaker from a distance and is according to a path
  public static final double k_goalRangeMeters = Units.inchesToMeters(194.3);

  // Aiming constants - NOT TUNED:
  public static final double k_angularP = 0;
  public static final double k_angularI = 0;
  public static final double k_angularD = 0;

  // Getting in range constants - NOT TUNED:
  public static final double k_forwardP = 0;
  public static final double k_forwardI = 0;
  public static final double k_forwardD = 0;

}
