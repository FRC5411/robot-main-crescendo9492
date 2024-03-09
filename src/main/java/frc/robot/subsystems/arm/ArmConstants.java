package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int k_armMotorID = 18;
    public static final int k_smartCurrentLimit = 40;
    public static final double k_armSpeed = 0.1;
    public static final double k_speedZero = 0.0;

    public static final double k_armGearRatio = (1.0 / 25.0) * (28.0 / 50.0) * (16.0 / 64.0);
    public static final double k_positionConversionFactor = k_armGearRatio;
    public static final double k_velocityConversionFactor = k_armGearRatio / 60.0;

    public static final double k_armP = 0.1; // 2.5 on REV ION
    public static final double k_armI = 0.0;
    public static final double k_armD = 0.0;

    public static final Rotation2d k_armEncoderOffset = Rotation2d.fromRadians(1.342);
    public static final double k_armFreeSpeed = 5676.0 * k_velocityConversionFactor;

    // Feed Forward, PID, and setpoint constants - TO BE CONFIGURED: 12,0 / k_armFreeSpeed
    public static final ArmFeedforward k_armFeedforward = new ArmFeedforward(0.0, 6.0, 0.0, 0.0);
    public static final ProfiledPIDController k_armPID = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    public static final double k_intakeSetpoint = 0.0;
    public static final double k_ampSetpoint = 0.171999;
    public static final double k_shootSetpoint = 0.0189998;
    public static final double k_tolerance = 0.1;

    public static final double k_armDeadband = 0.1;
    public static final double k_upperBound = 0.0 + k_armEncoderOffset.getRotations();
    public static final double k_lowerBound = -0.3 + k_armEncoderOffset.getRotations();
}
