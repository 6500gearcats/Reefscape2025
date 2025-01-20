// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

//TODO: Remove this suppress warnings after finishing
@SuppressWarnings("unused")

public class GCPoseEstimator extends SubsystemBase {

  //TODO: Add appropriate the standard deviation for the model's states and the vision measurements.

  // This uses the Kulman Filter to estimate the pose of the robot. 
  // HIGHLY RECOMMENDED to research the Kalman Filter to properly understand this.

  // You can tune Standard deviation of repective estimation/measurement to configure how much you trust them.
  // Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /*
  * The Standard Deviation for the Model's States.
  * Increase numbers to trust the model's state estimates less
  * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
  */
  private static final Vector<N3> m_stateStndDev =  VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));

  /* 
  * The Standard Deviation for the Vsion Measurements (i,e. NOSIE);
  * Increase numbers to trust mesurements from Vision less.
  * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
  */
  private static final Vector<N3> m_visionStndDev = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(0.1));

  
  private SwerveDrivePoseEstimator m_poseEstimator;
  private Supplier<Rotation2d> m_rotationSupplier;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;


  /** Creates a new PoseEstimator. */
  public GCPoseEstimator(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> swerveModulePositionSupplier) {
    m_rotationSupplier = rotationSupplier;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_rotationSupplier.get(),
      m_swerveModulePositionSupplier.get(),
      new Pose2d(),
      m_stateStndDev,
      m_visionStndDev);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());
    useLimeLight();
  }

  public void setVisionMeasurement(Pose2d pose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp);
  }
  public void setVisionStndDeviation(Vector<N3> visionStndDev) {
    m_poseEstimator.setVisionMeasurementStdDevs(visionStndDev);
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void useLimeLight() {
    //TODO: add logic to update useMegaTag2 dynamically based on what limelight sees
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(DriveSubsystem.m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }
}
