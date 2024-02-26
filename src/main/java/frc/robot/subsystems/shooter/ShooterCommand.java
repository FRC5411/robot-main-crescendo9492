// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class ShooterCommand extends SequentialCommandGroup {
  /** Creates a new ShooterCommand. */
  private Shooter shooter;
  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;
  private CANSparkMax indexerMotor;
  private CANSparkMax pivotMotor;
  private ArmFeedforward FF;
  private ProfiledPIDController PID;

  public ShooterCommand() {
    leftShooterMotor = new CANSparkMax(ShooterConstants.k_leftShooterMotorID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ShooterConstants.k_rightShooterMotorID, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(ShooterConstants.k_indexerMotorID, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ShooterConstants.k_pivotMotorID, MotorType.kBrushless);
    rightShooterMotor.follow(leftShooterMotor);
    FF = new ArmFeedforward(ShooterConstants.kS, ShooterConstants.kG, ShooterConstants.kV, ShooterConstants.kA);
    PID = new ProfiledPIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, new TrapezoidProfile.Constraints(ShooterConstants.k_armVelocity, ShooterConstants.k_armAcceleration));
    // addRequirements(shooter);
  }

  // Goes to idle position:
public void goToIdlePos() {
    double idlePIDCalc = PID.calculate(shooter.getPosition().getDegrees(), ShooterConstants.idleSetpoint);
    double idleFFCalc = FF.calculate(shooter.getPosition().getRadians(), shooter.getVelocity().getRadians());
    // Sets voltage for pivot motor
    shooter.setMotorVolts(idlePIDCalc + idleFFCalc);
  }

  // Sequential Command Group - Have motor for intake wheels start running first, then bring down to floor position
  public void goToIntakePos() {
    double floorPIDCalc = PID.calculate(shooter.getPosition().getDegrees(), ShooterConstants.intakeSetpoint);
    double floorFFCalc = FF.calculate(shooter.getPosition().getRadians(), shooter.getVelocity().getRadians());
    // Sets voltage for pivot motor
    shooter.setMotorVolts(floorPIDCalc + floorFFCalc);
    // Commands for Sequential Command Group
    addCommands(
      (new InstantCommand(() -> startIndexerMotor())),
      new InstantCommand(() -> goToIntakePos())
    );
  }


  // Temporary method to start shooter and intake wheels at the same time for testing purposes
  public void startIndexerMotor() {
    indexerMotor.set(ShooterConstants.k_indexerMotorSpeed);
    //leftShooterMotor.set(1.0);
    //rightShooterMotor.set(1.0);
  }

  // Shoot method:
  public void shoot() {
    //leftShooterMotor.set(ShooterConstants.k_shooterMotorSpeed);
    //rightShooterMotor.set(ShooterConstants.k_shooterMotorSpeed);
  }
  // Called when the command is initially scheduled.
 /*@Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }*/
}