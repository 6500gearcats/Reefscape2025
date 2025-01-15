// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveArgs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  //private final RelativeEncoder m_drivingEncoder;
  //private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, SwerveArgs args) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    SparkMaxConfig drivingConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    drivingConfig.encoder
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        .positionConversionFactor(args.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(args.kDrivingEncoderVelocityFactor);

    drivingConfig.closedLoop
        .pid(args.kDrivingP, args.kDrivingI, args.kDrivingD)
        .velocityFF(args.kDrivingFF)
        .outputRange(args.kDrivingMinOutput, args.kDrivingMaxOutput);
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.

        
    drivingConfig.smartCurrentLimit(args.kDrivingMotorCurrentLimit);

    drivingConfig.signals.primaryEncoderPositionPeriodMs(5);
    drivingConfig.idleMode(args.kDrivingMotorIdleMode);

    turningConfig.absoluteEncoder
        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        .positionConversionFactor(args.kTurningEncoderPositionFactor)
        .velocityConversionFactor(args.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    turningConfig.inverted(args.kTurningEncoderInverted)
        .idleMode(args.kTurningMotorIdleMode);

    turningConfig.closedLoop
        // Set the PID gains for the turning motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(args.kTurningP, args.kTurningI, args.kTurningD)
        .velocityFF(args.kTurningFF)
        .outputRange(args.kTurningMinOutput, args.kTurningMaxOutput)
        
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(args.kTurningEncoderPositionPIDMinInput)
        .positionWrappingMaxInput(args.kTurningEncoderPositionPIDMaxInput);

    turningConfig.smartCurrentLimit(args.kTurningMotorCurrentLimit);

    turningConfig.signals.primaryEncoderPositionPeriodMs(5);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.

    
    // public ClosedLoopConfig setPositionPIDWrappingEnabled(boolean enabled) {
    //   m_positionPIDWrappingEnabled = enabled;
    //   return this;
    // }


    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition());
    //m_drivingEncoder.setPosition(0);

    // REVPhysicsSim.getInstance().addSparkMax(m_drivingSparkMax, DCMotor.getNEO(1));
    // REVPhysicsSim.getInstance().addSparkMax(m_turningSparkMax, DCMotor.getNEO(1));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingSparkMax.getEncoder().getVelocity(),
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      m_drivingSparkMax.getEncoder().getPosition(),
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingSparkMax.getEncoder().setPosition(0);
  }

  public void setCoast() {
  }
}
