// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

//TODO: Remove this suppress warnings after finishing
@SuppressWarnings("unused")

public class GCPoseEstimator extends SubsystemBase {

  //TODO: Add appropriate the standard deviation for the model's states and the vision measurements.

  /* 
  * This uses the Kulman Filter to estimate the pose of the robot. 
  * HIGHLY RECOMMENDED to research the Kalman Filter to properly understand this.
  */

  /*
  * You can tune Standard deviation of repective estimation/measurement to configure how much you trust them.
  * Smaller numbers will cause the filter to
  * "trust" the estimate from that particular component more than the others. 
  * This in turn means the particualr component will have a stronger influence
  * on the final pose estimate.
  */

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
  private boolean m_useLimeLight;
  private DriveSubsystem m_robotDrive;

  private Vision m_vision;


  /** Creates a new PoseEstimator. */
  // * Uses Limelight
  public GCPoseEstimator(DriveSubsystem m_robotDrive, Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> swerveModulePositionSupplier) {
    this.m_robotDrive = m_robotDrive;
    m_rotationSupplier = rotationSupplier;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;
    m_useLimeLight = true;

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_rotationSupplier.get(),
      m_swerveModulePositionSupplier.get(),
      new Pose2d(),
      m_stateStndDev,
      m_visionStndDev);
  }

  // * Uses PhotonVision
  public GCPoseEstimator(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> swerveModulePositionSupplier, Vision vision) {
    m_rotationSupplier = rotationSupplier;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;
    m_useLimeLight = false;
    m_vision = vision;

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
    if(m_useLimeLight) {
      useLimeLight();
    }
    else {
      usePhotonVision();
    }
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  //The following method is limelight integration from LimeLight themselves.
  public void useLimeLight() {
    //TODO: add logic to update useMegaTag2 dynamically based on what limelight sees
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;

    if(useMegaTag2 == false)
    {
      /*
      * In 2024, most of the WPILib Ecosystem transitioned to a single-origin coordinate system. 
      * For 2024 and beyond, the origin of your coordinate system should always be the "blue" origin.
      * FRC teams should always use botpose_wpiblue for pose-related functionality
      */
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-gcc");
      // Optional<Alliance> alliance = DriverStation.getAlliance();
      //   if (alliance.isPresent()) {
      //       if (alliance.get() == Alliance.Red) {
      //         mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-gcc");
      //       }
      //     else {
      //       mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-gcc");
      // }
      if (mt1!=null) {
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
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5, Units.degreesToRadians(20)));
          m_poseEstimator.addVisionMeasurement(
              mt1.pose,
              mt1.timestampSeconds);
        }
      }
    }
    else if (useMegaTag2 == true)
    {
      /*
      * In 2024, most of the WPILib Ecosystem transitioned to a single-origin coordinate system. 
      * For 2024 and beyond, the origin of your coordinate system should always be the "blue" origin.
      * FRC teams should always use botpose_wpiblue for pose-related functionality
      */
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if (alliance.get().equals(Alliance.Blue)) {
          LimelightHelpers.SetRobotOrientation("limelight-gcc", m_robotDrive.getAngle(), 0, 0, 0, 0, 0);
        } else {
          LimelightHelpers.SetRobotOrientation("limelight-gcc", m_robotDrive.getAngle() + 180, 0, 0, 0, 0, 0);
        }
      }
      LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.318, 0.177, 0.29, 0, 0, 180);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-gcc");
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

// * The following method is PhotonVision integration
// TODO: add simualtion support for PhotonVision
  public void usePhotonVision() {
    // var visionEst = m_vision.getEstimatedGlobalPose();
    // visionEst.ifPresent(est -> {
    //   var estStdDevs = m_vision.getEstimationStdDevs(est.estimatedPose.toPose2d());
    //   m_poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    // });
  }
}
