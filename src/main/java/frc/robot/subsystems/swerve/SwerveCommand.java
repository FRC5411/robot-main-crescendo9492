package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveCommand extends Command {

    private Swerve robotSwerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private Boolean robotCentricSup;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter rotationLimiter;

    public SwerveCommand(Swerve robotSwerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {

        this.robotSwerve = robotSwerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        this.robotCentricSup = robotCentricSup;

        translationLimiter = new SlewRateLimiter(3.0);
        strafeLimiter = new SlewRateLimiter(3.0);
        rotationLimiter = new SlewRateLimiter(8.0);

        addRequirements(robotSwerve);
    }

    @Override
    public void execute() {
        double translationVal = translationLimiter.calculate(
                MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.stickDeadband));
        double strafeVal = strafeLimiter.calculate(
                MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.stickDeadband));
        double rotationVal = rotationLimiter.calculate(
                MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.stickDeadband));

        robotSwerve.swerveDrive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed),
                rotationVal * SwerveConstants.maxAngularVelocity,
                !robotCentricSup,
                true);

        robotSwerve.odom(translationSup, rotationSup);
    }
}