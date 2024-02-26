package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static final int k_armMotorID = 18;
    public static final int k_smartCurrentLimit = 40;
    public static final double k_armSpeed = 0.5;

    public static final double k_armDeadband = 0.2;
    public static final double k_upperBound = 0.0;
    public static final double k_lowerBound = -0.3;
    public static final double k_softBound = 0.0;

    public static final double k_armGearRatio = (1.0 / 25.0) * (28.0 / 50.0) * (16.0 / 64.0);
    public static final double k_positionConversionFactor = k_armGearRatio;

    public static final double k_armP = 0.1;
    public static final double k_armI = 0.0;
    public static final double k_armD = 0.0;

    public static final Rotation2d k_armEncoderOffset = Rotation2d.fromRotations(0.0);
    public static final double k_velocityFactor = k_armGearRatio * 2.0 * Math.PI / 60.0;
    public static final double k_armFreeSpeed = 5676.0 * k_velocityFactor;
    public static final ArmFeedforward k_armFeedforward =
        new ArmFeedforward(0.0, 3.0, 12.0 / k_armFreeSpeed, 0.0);

}
