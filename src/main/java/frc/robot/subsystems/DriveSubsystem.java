// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
// Path Planner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.Vision;

public class DriveSubsystem extends SubsystemBase {
  // ! Update this to use the pose estimator instead of normal odametry

  public boolean turboEnable = false;
  public int aprilTag = 0;
  public int aprilTagDrive = 0;
  public Pose2d aprilTagPose = new Pose2d();
  public Pose2d aprilTagPose2 = new Pose2d();
  private AprilTagFieldLayout field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

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

  //The new Pose Estimator
  private GCPoseEstimator m_poseEstimator;

  private final Field2d m_field = new Field2d();

  private GCPhotonVision m_simVision;
  // ! Temporarily added this to make the pose estimator
  private Vision m_vision;

  private Pose2d m_simOdometryPose;
  private ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  private GenericEntry m_maxSpeed;
  
  /*public SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_rearLeft.getPosition(),
    m_rearRight.getPosition()
}, new Pose2d(0.0, 0.0, new Rotation2d()));*/

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
    if(m_vision.usingLimelight()){
      m_poseEstimator = new GCPoseEstimator(this, this::getRotation2d, this::getWheelPositions);
    }
    else{
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
        "max", DriveConstants.kTurboModeModifier*2)) // specify widget properties here
      .getEntry();

    
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

      AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeed,
        (speeds, feedforwards) -> drive(speeds),
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
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
    this
);
publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

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
    SmartDashboard.putNumber("Integer of april tag: ", aprilTag);
    aprilTagDrive = getClosestAprilTagID(getPose().getTranslation());
    //aprilTagPose = field.getTagPose(aprilTagDrive).get().toPose2d();
    //aprilTagPose = getBestAprilTag();
    //aprilTagPose2 = getBestAprilTag2();
    
    SmartDashboard.putNumber("Integer of april tag Drive: ", getClosestAprilTagID(getPose().getTranslation()));

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

  @Override
  public void simulationPeriodic() {
    // Update the odometry in the periodic block
    
    //SparkSim.iterate();

    
    
    // Update camera simulation
    m_simVision.simulationPeriodic(this.getPose());

    var debugField = m_simVision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(this.getPose());
    // debugField.getObject("EstimatedRobotModules").setPoses(this.getModulePoses());

    // angle.set(5.0);
    // From NavX example
    // int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    // SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,
    // "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative

    // navxSimAngle = -drivetrainSim.getHeading().getDegrees();
    // double angle = getPose().getRotation().getDegrees();

    // double angle = m_gyro.getAngle() -
    // Math.toDegrees(m_lastSpeeds.omegaRadiansPerSecond) * 0.20 ;
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
      //     m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
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

    m_fieldOriented = fieldRelative;
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kNormalSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kNormalSpeedMetersPerSecond;

    rot *= DriveConstants.kMaxAngularSpeed;
    // Non linear speed set
    // xSpeed *= Math.signum(xSpeed)*Math.pow(xSpeed,3);
    // ySpeed *= Math.signum(ySpeed)*Math.pow(ySpeed,3);

    double max = m_maxSpeed.getDouble(DriveConstants.kTurboModeModifier);
    
    if(Elevator.elevatorTooHighForTurbo){
      if(Elevator.elevatorTooHighForRegularSpeed){
        xSpeed *= .5;
        ySpeed *= .5;
        rot *= .5;
      }
    } else if (turboEnable) {
      
      xSpeed *= max;
      ySpeed *= max;
      rot *= DriveConstants.kTurboAngularSpeed;
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
      return m_simAngle.get();
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

  //Code Added for pose estimator- Pranav
  public Rotation2d getRotation2d(){
    return m_gyro.getRotation2d();
  }

  public SwerveModulePosition[] getWheelPositions(){
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  public Pose2d getBestAprilTag() {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    Pose2d pose = getPose();
    int bestAprilTag = getClosestAprilTagID(pose.getTranslation());
    Pose2d newPose = field.getTagPose(bestAprilTag).get().toPose2d();
    System.out.println("Old Poses values" + newPose.getX() + ", " + newPose.getY() + ". Rotation: " + newPose.getRotation());

    double tempAngle = field.getTagPose(bestAprilTag).get().toPose2d().getRotation().getRadians();
    double newX = 0;
    double newY = 0;
    newX = (newPose.getX() + Math.cos(tempAngle) * .66) + Math.cos(tempAngle + Math.PI/2) * .3;
    newY = (newPose.getY() + Math.sin(tempAngle) * .66) + Math.sin(tempAngle + Math.PI/2) * .3;
    Pose2d thirdPose = new Pose2d(newX, newY, newPose.getRotation().plus(new Rotation2d(Math.PI)));
    System.out.println("New Poses values" + thirdPose.getX() + ", " + thirdPose.getY() + ". Rotation: " + thirdPose.getRotation());
    return thirdPose;
  }

  public Pose2d getBestAprilTag2() {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    Pose2d pose = getPose();
    int bestAprilTag = getClosestAprilTagID(pose.getTranslation());
    Pose2d newPose = field.getTagPose(bestAprilTag).get().toPose2d();
    System.out.println("Old Poses values" + newPose.getX() + ", " + newPose.getY() + ". Rotation: " + newPose.getRotation());

    double tempAngle = field.getTagPose(bestAprilTag).get().toPose2d().getRotation().getRadians();
    double newX = 0;
    double newY = 0;
    newX = (newPose.getX() + Math.cos(tempAngle) * .66) - Math.cos(tempAngle + Math.PI/2) * .3;
    newY = (newPose.getY() + Math.sin(tempAngle) * .66) - Math.sin(tempAngle + Math.PI/2) * .3;
    Pose2d thirdPose = new Pose2d(newX, newY, newPose.getRotation().plus(new Rotation2d(Math.PI)));
    System.out.println("New Poses values" + thirdPose.getX() + ", " + thirdPose.getY() + ". Rotation: " + thirdPose.getRotation());
    return thirdPose;
  }

  public int getClosestAprilTagID(Translation2d robotPose) {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    int integer = 0;
    ArrayList<Double> poses = new ArrayList<Double>();
    ArrayList<Integer> list = new ArrayList<Integer>();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red) {
          for (int i = 6; i<12; i++) {
            Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
            double distance = robotPose.getDistance(pose);
            poses.add(distance);
            list.add(i);
          }
        }
        if (alliance.get() == Alliance.Blue) {
          double distance = (double)Integer.MAX_VALUE;
          for (int i = 17; i<23; i++) {
            Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
            if(Math.abs(robotPose.getDistance(pose)) < distance){
              distance = robotPose.getDistance(pose);
              integer = i;
            }
            //distance = robotPose.getDistance(pose);
            //poses.add(distance);
            //list.add(i);
          }
        }
    }
    else {
      for (int i = 1; i<23; i++) {
        Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
        double distance = robotPose.getDistance(pose);
        poses.add(distance);
        list.add(i);
      }
    }
    //double minValue = Collections.min(poses);
    //System.out.println("Min value: " + minValue);
    //integer = posesfp.indexOf(minValue);
    //System.out.println("AprilTag of least distance: " + list.get(integer));
    aprilTagDrive = integer;//list.get(integer);
    return integer;
  }
}
