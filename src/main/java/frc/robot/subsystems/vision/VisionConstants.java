// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class VisionConstants {

    // FIND MEASUREMENTS ONCE CAMERA IS MOUNTED:
    public static final double k_cameraHeightMeters = 0;
    public static final double k_targetHeightMeters = 0;
    public static final double k_cameraPitchRadians = 0;

    // This measurement is if we have the fixed shooter height be made specifically
    // for shooting into the speaker from a distance and is according to a path
    public static final double k_goalRangeMeters = Units.inchesToMeters(194.3);

    // Aiming constants - NOT TUNED:
    public static final double angularP = 0;
    public static final double angularI = 0;
    public static final double angularD = 0;

    // Getting in range constants - NOT TUNED:
    public static final double forwardP = 0;
    public static final double forwardI = 0;
    public static final double forwardD = 0;
  }
}
