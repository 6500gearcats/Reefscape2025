// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.6; // 4.5
    public static final double kNormalSpeedMetersPerSecond = 1.5; // 0.85
    public static final double kMaxAngularSpeed = 1 * Math.PI; // radians per second (was 0.75)

    // turbo
    public static final double kTurboModeModifier = 7.0;
    public static double kTurboAngularSpeed = 2.0;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // caclulate front wheel offset angles using math. Similar angles means we can
    // just use width/length as opposite/adjacent
    public static final double theta = Math.atan((kTrackWidth) / (kWheelBase));

    public static final double kFrontLeftChassisAngularOffset = -theta * 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI - 2 * theta;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static RobotConfig config;
    static {
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
    }

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // These constants are not correct, temporary until parts added.

  public static final class NeckConstants {
    // These are port values (where it's located on the robot)
    public static final int kNeckMotorPort = 11;
    // NOT UPDATED TO 2024 NECK VALUES
    public static final double kNeckReverseSpeed = -0.4;
    public static final double kNeckForwardSpeed = 0.6; // Was 0.6
    public static final double kNeckForwardMaxSpeed = 0.15;
    public static final double kNeckReverseMaxSpeed = -0.15;
    public static final double kNeckStableSpeed = 0.058;

    // TODO tune
    public static final double kNeckSlowModifier = 0.57;

    // NOT UPDATED TO 2024 NECK VALUES
    public static final double kEncoderUpperThreshold = 0.30;
    public static final double kEncoderLowerThreshold = 0.01;
    public static final double KEncoderDeadbandThreshold = 0.01;
    public static final double kNeckStowAngle = 0.8;
    public static final double kNeckFloorAngle = 0;
    public static final double kNeckHighAngle = 0.315; // good
    public static final double kNeckMidAngle = 0.37; // good
    public static final double kNeckLowAngle = 0.70;
    public static final double kLoadingStation = 0.36;

    // Controller constants
    public static final double kNeck_kS = 1.7;
    public static final double kNeck_kG = 0.5;
    public static final double kNeck_kV = 0.0;

    public static final double kNeck_kP = 0.0;
    public static final double kNeck_kI = 0.0;
    public static final double kNeck_kD = 0.0;
    public static final double kNeck_kP2 = 1;
    public static final double kNeck_kI2 = 0;
    public static final double kNeck_kD2 = 0;

  }

  public static final class GyroConstants {
    public static final double kTiltPitch = 65; // 11? tilt angle=
  }

  public static class Vision {
    public static final String kCameraName = "YOUR CAMERA NAME";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
public static class VisionConstants {
    public static final String kCameraNameTag = "Microsoft_LifeCam_HD-3000";
    public static final String kCameraNameNote = "Microsoft_LifeCam_VX-5000";
    public static final String kCameraNameGlobal = "Global_Shutter_Camera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(27);

    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-28);

    // How far from the target we want to be
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  }

  public static class ShootNoteConstants {
    public static final long kmiliSeconds = 1000;
  }

  public static class ShooterConstants {
    public static final int kShooterMotorPort = 10;

    // NOT UPDATED TO 2024 SHOOTER VALUES (besides port)
    public static final double kShooterSpeedSlow = 0.6;
    public static final double kBackwardsShooter = 0.2;
    public static final double kShooterFastRPM = 2100;
    // public static final int kShooterSlowRPM = 3000;
    public static final int kDistanceShooterRPM = 2500; // Was 3000
    public static final int kShooterTrapRPM = 3000;
    public static final double kShooterSpeedFast = -0.6;
    public static final double kDistanceShooterSpeedFast = -0.8; // was -0.75
    public static final double kShooterReverseFast = 1;
    public static final double kShooterSpeed = 1.1; // 0.43
    public static final double kBallFiredThreshold = 0.1;
    public static final int kShooterEncoderPort = 10;
    public static final double kShooterDistanceFactor = 12.391;

    public static final double kRangeAngleError = 0.02;
  }

  public static class ClimberConstants {
    public static final int kRight_ClimberMotorPort = 13;
    public static final int kLeft_ClimberMotorPort = 12;
    public static final double kMaxDriveSpeed = 0.1;
    public static final double kMaxArmHeight = 7.7;
    public static final double kMinArmHeight = 0.1; // Random filler number
    public static final double kClimberSpeed = 0;
    public static final double kClimberSpeedUp = 0.75;
    public static final double kClimberSpeedDown = -0.6;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorPort = 9;
    public static final double kFeedSpeed = 0.50;
    public static final double kReverseFeedSpeed = -0.1;
    public static final double kPickUpSpeed = 0.6;
    public static final double kPickUpSpeedSlow = 0.2;
  }

  // values

  public static final double kRangeSpeedOffset = 0.6;
  public static final double ANGULAR_P = 0.1;
  public static final double ANGULAR_D = 0.0;
  public static final double kDefaultNoteFinderSpeed = 1;
}

//
