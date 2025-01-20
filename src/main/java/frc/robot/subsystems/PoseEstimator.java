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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

//TODO: Remove this suppress warnings after finishing
@SuppressWarnings("unused")

public class PoseEstimator extends SubsystemBase {

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
  private static final Vector<N3> stateStndDev =  VecBuilder.fill(0.1, 0.1, 0.1);

  /* 
  * The Standard Deviation for the Vsion Measurements (i,e. NOSIE);
  * Increase numbers to trust mesurements from Vision less.
  * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
  */
  private static final Vector<N3> visionStndDev = VecBuilder.fill(0.1,0.1,0.1);

  /** Creates a new PoseEstimator. */
  private SwerveDrivePoseEstimator m_poseEstimator;
  private Supplier<Rotation2d> m_rotationSupplier;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;

  public PoseEstimator(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> swerveModulePositionSupplier) {
    m_rotationSupplier = rotationSupplier;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_rotationSupplier.get(),
      m_swerveModulePositionSupplier.get(),
      new Pose2d(),
      stateStndDev,
      visionStndDev);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
