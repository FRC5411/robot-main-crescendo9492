// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera camera;


  
  PIDController aimingController;
  PIDController rangeController;

  XboxController controller;

  // These are temporary and would be replaced with the drive motors
  // Declared in the drive subsystem
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  double forward;
  double theta;

  private DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);
  
  public Vision() {
    camera = new PhotonCamera("camera");
    controller = new XboxController(1);
    backLeft.follow(frontLeft, false);
    backRight.follow(frontRight, false);

    rangeController = new PIDController(VisionConstants.k_forwardP, VisionConstants.k_forwardI, VisionConstants.k_forwardD);
    aimingController = new PIDController(VisionConstants.k_angularP, VisionConstants.k_angularI, VisionConstants.k_angularD);

  }

  public void aimAndRange() {
    if(controller.getAButton()) {

      var result = camera.getLatestResult();

      double targetPitchRadians = Math.toRadians(result.getBestTarget().getPitch());

      if(result.hasTargets()) {
        
        double range = 
        PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.k_cameraHeightMeters, VisionConstants.k_targetHeightMeters, VisionConstants.k_cameraPitchRadians, targetPitchRadians);

        forward = -rangeController.calculate(range, VisionConstants.k_goalRangeMeters);
        theta = -aimingController.calculate(result.getBestTarget().getYaw(), 0);

        drive.arcadeDrive(forward, theta);

      }
      else {
        forward = 0;
        theta = 0;
      }
    }

    else {
      forward = controller.getLeftY();
      theta = controller.getRightX();

      drive.arcadeDrive(forward, theta);
    }
   
  
  }
  

  @Override
  public void periodic() {
    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(null, null, forward, theta, forward, null);
  }
}
