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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private static final Vector<N3> m_stateStndDev =  VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10));

  /* 
  * The Standard Deviation for the Vsion Measurements (i,e. NOSIE);
  * Increase numbers to trust mesurements from Vision less.
  * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
  */
  private static final Vector<N3> m_visionStndDev = VecBuilder.fill(0.7,0.7,Units.degreesToRadians(9999));

  
  private SwerveDrivePoseEstimator m_poseEstimator;
  private Supplier<Rotation2d> m_rotationSupplier;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private boolean m_useLimeLight;

  private Vision m_vision;
  private String m_limelightName1;
  private String m_limelightName2;

  private String CLL = "";


  /** Creates a new PoseEstimator. */
  // * Uses Limelight
  public GCPoseEstimator(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> swerveModulePositionSupplier, Vision m_vision) {
    m_rotationSupplier = rotationSupplier;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;
    m_useLimeLight = true;
    this.m_vision = m_vision;

    m_limelightName1 = m_vision.getNameLL1();
    m_limelightName2 = m_vision.getNameLL2();

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_rotationSupplier.get(),
      m_swerveModulePositionSupplier.get(),
      new Pose2d(),
      m_stateStndDev,
      m_visionStndDev);
  }

  // * Uses PhotonVision
  /**
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
  */

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
    SmartDashboard.putString("Current limelight", CLL);
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  //The following method is limelight integration from LimeLight themselves.
  public void useLimeLight() {
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    // if(useMegaTag2 == false)
    // {
    //   /*
    //   * In 2024, most of the WPILib Ecosystem transitioned to a single-origin coordinate system. 
    //   * For 2024 and beyond, the origin of your coordinate system should always be the "blue" origin.
    //   * FRC teams should always use botpose_wpiblue for pose-related functionality
    //   */

    //   LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName1);
    //   LimelightHelpers.PoseEstimate mt1_2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName2);
      
    //   if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    //   {
    //     if(mt1.rawFiducials[0].ambiguity > .7)
    //     {
    //       doRejectUpdate = true;
    //     }
    //     if(mt1.rawFiducials[0].distToCamera > 3)
    //     {
    //       doRejectUpdate = true;
    //     }
    //   }
    //   if(mt1.tagCount == 0)
    //   {
    //     doRejectUpdate = true;
    //   }

    //   if(!doRejectUpdate)
    //   {
    //     m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
    //     m_poseEstimator.addVisionMeasurement(
    //         mt1.pose,
    //         mt1.timestampSeconds);

    //     m_poseEstimator.addVisionMeasurement(
    //         mt1_2.pose,
    //         mt1_2.timestampSeconds + 0.001);
    //   }
    // }
    if (useMegaTag2 == true)
    {
      /*
      * In 2024, most of the WPILib Ecosystem transitioned to a single-origin coordinate system. 
      * For 2024 and beyond, the origin of your coordinate system should always be the "blue" origin.
      * FRC teams should always use botpose_wpiblue for pose-related functionality
      */
      LimelightHelpers.SetRobotOrientation(m_limelightName1, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(m_limelightName2, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate currentMt2;

      // * Following code prefers mt2 becuase we presume it is the more powerfull limelight of the two
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName1);
      currentMt2 = mt2;

      LimelightHelpers.PoseEstimate mt2_2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName2);

      Pose2d prevPose = new Pose2d();
      
      if(mt2 != null && mt2_2 != null) {
        if(Math.abs(DriveSubsystem.m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }
        if(mt2.tagCount == 0 || mt2_2.tagCount == 0)
        {
          // * Switches between limeights based on what can actually see april tags
          if(mt2.tagCount == 0 ) {
            currentMt2 = mt2_2;
            CLL = "LL3a";
          }
          else if (mt2_2.tagCount == 0 ) {
            currentMt2 = mt2;
            CLL = "LL4";
          }
          else {
            doRejectUpdate = true;
          }
        }
        // ! Change the threshold based on what you want the cutoff to be 
        if((Math.abs(currentMt2.pose.getX() - prevPose.getX()) > 5 || Math.abs(currentMt2.pose.getY() - prevPose.getY()) > 5 ) && (currentMt2.timestampSeconds < 300)) {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
          // * Switches between limeights based on which one can see more april tags 
          if(mt2.tagCount >= mt2_2.tagCount) {
            currentMt2 = mt2;
            CLL = "LL4";
          }
          else {
            currentMt2 = mt2_2;
            CLL = "LL3a";
          }
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
          m_poseEstimator.addVisionMeasurement(
              currentMt2.pose,
              currentMt2.timestampSeconds);
          
          prevPose = currentMt2.pose;
        }
      }
    }
  }

// * The following method is PhotonVision integration
// TODO: add simualtion support for PhotonVision
  public void usePhotonVision() {
    /**var visionEst = m_vision.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
      var estStdDevs = m_vision.getEstimationStdDevs(est.estimatedPose.toPose2d());
      m_poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
  } */
  }
}
