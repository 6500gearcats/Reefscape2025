// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;
// Path Planner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.GCPhotonVision;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  // ! Update this to use the pose estimator instead of normal odametry

  public boolean turboEnable = false;
  public boolean snailEnable = false;

  // public Pose2d aprilTagPose = new Pose2d();
  // public Pose2d aprilTagPose2 = new Pose2d();
  public double targetrotation = 0;

  // Proportional alignment logging values
  private double distanceX = 0;
  private double distanceY = 0;
  private double distanceR = 0;
  private double fakeYaw = 0;
  
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public static Pigeon2 m_gyro;
  public static AHRS m_gyro2;

  private int m_gyroSim;
  private SimDouble m_simAngle;
  private SimBoolean m_connected;
  private SimBoolean m_calibrating;
  private boolean m_fieldOriented;

  private ChassisSpeeds m_lastSpeeds;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  // The new Pose Estimator
  private GCPoseEstimator m_poseEstimator;

  private final Field2d m_field = new Field2d();

  private GCPhotonVision m_simVision;
  // ! Temporarily added this to make the pose estimator
  private Vision m_vision;
  private String systemControlling = "";

  private Pose2d m_simOdometryPose;
  private ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  private GenericEntry m_maxSpeed;

  private double commandedXSpeed = 0.0;
  private double commandedYSpeed = 0.0;
  private double commandedRotation = 0.0;

  /*
   * public SwerveDrivePoseEstimator m_poseEstimator = new
   * SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
   * m_gyro.getRotation2d(), new SwerveModulePosition[] {
   * m_frontLeft.getPosition(),
   * m_frontRight.getPosition(),
   * m_rearLeft.getPosition(),
   * m_rearRight.getPosition()
   * }, new Pose2d(0.0, 0.0, new Rotation2d()));
   */

  private final StructArrayPublisher<SwerveModuleState> publisher;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(GCPhotonVision simVision, Vision vision) {
    m_simVision = simVision;
    // ! Temporarily added this to make the pose estimator
    m_vision = vision;
    try {
      /*
       * Communicate w/navX-MXP via theimport com.kauailabs.navx.frc.AHRS;A MXP SPI
       * Bus.
       */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      m_gyro2 = new AHRS(NavXComType.kMXP_SPI);
      m_gyro = new Pigeon2(30, "rio");
      System.out.println("Pigeon2 constructed");
    } catch (RuntimeException ex) {
      System.out.println("Pigeon2 not constructed");
      DriverStation.reportError("Error instantiating Pigeon2:  " + ex.getMessage(), true);
    }

    // m_gyro.setAngleAdjustment(180.0);
    m_gyro.setYaw(0);
    m_gyro.setYaw(0);

    if (RobotBase.isSimulation()) {
      m_gyroSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      m_simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Yaw"));
      m_connected = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Connected"));
      m_calibrating = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Calibrating"));
      m_connected.set(true);
      m_calibrating.set(false);
      SmartDashboard.putNumber(getName(), getPitch());
    }

    // * Create a new PoseEstimator
    if (m_vision.usingLimelight()) {
      m_poseEstimator = new GCPoseEstimator(this, this::getRotation2d, this::getWheelPositions);
    } else {
      m_poseEstimator = new GCPoseEstimator(this::getRotation2d, this::getWheelPositions, m_vision);
    }

    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, new Pose2d(0.0, 0.0, new Rotation2d()));

    m_lastSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Rotation2d.fromDegrees(0.0));

    m_simOdometryPose = m_odometry.getPoseMeters();
    SmartDashboard.putData("Field", m_field);

    m_maxSpeed = m_driveTab.add("Max Speed", DriveConstants.kTurboModeModifier)
        .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
        .withProperties(Map.of(
            "min", DriveConstants.kTurboModeModifier,
            "max", DriveConstants.kTurboModeModifier * 2)) // specify widget properties here
        .getEntry();

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeed,
        (speeds, feedforwards) -> drive(speeds),
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(10.0, 0.0, 0.0) // Rotation PID constants
        ),
        AutoConstants.config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

  }

  @Override
  public void periodic() {
    // Update the odometry in te periodic block
    updateOdometry();

    SmartDashboard.putNumber("target rotation", targetrotation);

    SmartDashboard.putString("System Running Drive", systemControlling);

    SmartDashboard.putNumber("Commanded X Speed", commandedXSpeed);
    SmartDashboard.putNumber("Commanded Y Speed", commandedYSpeed);
    SmartDashboard.putNumber("Commanded Rotation", commandedRotation);

    if (Robot.isReal()) {
      m_field.setRobotPose(getPose());
    } else {
      m_field.setRobotPose(m_simOdometryPose);

      Pose2d CurrentPos = m_simOdometryPose;
      double xPos = CurrentPos.getX();
      double yPos = CurrentPos.getY();
      SmartDashboard.putNumber("Position: X", xPos);
      SmartDashboard.putNumber("Position: Y", yPos);
    }

    SmartDashboard.putNumber("Pigeon2 Pitch", m_gyro.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon2 Yaw angle", getAngle());
    SmartDashboard.putNumber("Pigeon2 Yaw angle radians", Math.toRadians(getAngle()));

    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);

    publisher.set(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    });

  }

  public void setYaw(int yaw) {
    m_gyro.setYaw(180);
  }

  @Override
  public void simulationPeriodic() {
    // Update the odometry in the periodic block

    // SparkSim.iterate();

    // Update camera simulation
    m_simVision.simulationPeriodic(this.getPose());

    var debugField = m_simVision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(this.getPose());

    double angle = m_simOdometryPose.getRotation().getDegrees();

    double newangle = Math.IEEEremainder(angle, 360);
    m_simAngle.set(newangle);

    SmartDashboard.putNumber("SimAngle", m_simAngle.get());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    if (Robot.isReal()) {
      return m_poseEstimator.getEstimatedPosition();
    } else {
      return m_simOdometryPose;
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

    m_simOdometryPose = pose;
  }

  // Dependency for the AutoBuilder.ConfigureHolonomic method
  public ChassisSpeeds getChassisSpeed() {
    return m_lastSpeeds;
  }

  /**
   * Updates the odometry of the robot using the swerve module states and the gyro
   * reading. Should
   * be run in periodic() or during every code loop to maintain accuracy.
   */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if (Robot.isSimulation()) {
      // SwerveModuleState[] measuredStates = new SwerveModuleState[] {
      // m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(),
      // m_rearRight.getState()
      // };
      // ChassisSpeeds speeds =
      // DriveConstants.kDriveKinematics.toChassisSpeeds(measuredStates);
      ChassisSpeeds speeds = m_lastSpeeds;

      Twist2d twist = new Twist2d(
          speeds.vxMetersPerSecond * .02,
          speeds.vyMetersPerSecond * .02,
          speeds.omegaRadiansPerSecond * .02);

      m_simOdometryPose = m_simOdometryPose.exp(twist);

      SmartDashboard.putNumber("new x", twist.dx);
      SmartDashboard.putNumber("new y ", twist.dy);
      SmartDashboard.putNumber("new theta ", twist.dtheta);

    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    this.drive(xSpeed, ySpeed, rot, fieldRelative, "auto");

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String system) {
    int inverse = 1;
    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // if (alliance.get().equals(Alliance.Blue)) {
    // inverse = 1;
    // } else {
    // inverse = -1;
    // }
    // }

    systemControlling = system;

    xSpeed *= inverse;
    ySpeed *= inverse;

    m_fieldOriented = fieldRelative;
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kNormalSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kNormalSpeedMetersPerSecond;

    rot *= DriveConstants.kMaxAngularSpeed;
    // Non linear speed set
    // xSpeed *= Math.signum(xSpeed)*Math.pow(xSpeed,3);
    // ySpeed *= Math.signum(ySpeed)*Math.pow(ySpeed,3);

    double max = m_maxSpeed.getDouble(DriveConstants.kTurboModeModifier);
    double min = .2;

    if (Elevator.elevatorTooHighForTurbo) {
      if (Elevator.elevatorTooHighForRegularSpeed) {
        xSpeed *= .5;
        ySpeed *= .5;
        rot *= .5;
      }
    } else if (turboEnable) {

      xSpeed *= max;
      ySpeed *= max;
      rot *= DriveConstants.kTurboAngularSpeed;
    }
    if (snailEnable) {
      xSpeed *= min;
      ySpeed *= min;
      rot *= 0.4;
    }

    m_lastSpeeds = (fieldRelative)
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_lastSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    fakeYaw += rot;

    commandedXSpeed = xSpeed;
    commandedYSpeed = ySpeed;
    commandedRotation = rot;

  }

  public void drive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    m_lastSpeeds = speeds;

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // if (alliance.get().equals(Alliance.Blue)) {
    // m_gyro.reset();

    // } else {
    // m_gyro.setYaw(180);
    // }
    // }
    m_gyro.reset();

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /* Return the NavX pitch angle */
  public double getPitch() {
    return m_gyro.getPitch().getValueAsDouble();
  }

  /* Return the NavX yaw angle */
  public double getAngle() {
    // return -m_gyro.getYaw();
    if (Robot.isReal()) {
      return m_gyro.getYaw().getValueAsDouble();
    } else {
      return fakeYaw * 1.169;
    }
  }

  public boolean toggleFieldOriented() {
    m_fieldOriented = !m_fieldOriented;
    return m_fieldOriented;
  }

  // PID Controllers
  public static PIDController turnController = new PIDController(Constants.ANGULAR_P, 0, Constants.ANGULAR_D);

  public void setDriveCoast() {
    m_frontLeft.setCoast();
    m_rearLeft.setCoast();
    m_frontRight.setCoast();
    m_rearRight.setCoast();
  }

  // Code Added for pose estimator- Pranav
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public SwerveModulePosition[] getWheelPositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }
}
