package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveArgs {

    //Simulation
    public Boolean simulation;


    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public double kMaxSpeedMetersPerSecond = 4.6; // 4.5
    public double kNormalSpeedMetersPerSecond = 1.5; // 0.85
    public double kMaxAngularSpeed = 1 * Math.PI; // radians per second (was 0.75)

    // turbo
    public double kTurboModeModifier = 7.0;
    public double kTurboAngularSpeed = 2.0;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public double kWheelBase = Units.inchesToMeters(28.5);

    public SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // caclulate front wheel offset angles using math. Similar angles means we can
    // just use width/length as opposite/adjacent
    public double theta = Math.atan((kTrackWidth) / (kWheelBase));

    public double kFrontLeftChassisAngularOffset = -theta * 2;
    public double kFrontRightChassisAngularOffset = 0;
    public double kRearLeftChassisAngularOffset = Math.PI;
    public double kRearRightChassisAngularOffset = Math.PI - 2 * theta;

    // SPARK MAX CAN IDs
    public int kFrontLeftDrivingCanId = 1;
    public int kRearLeftDrivingCanId = 4;
    public int kFrontRightDrivingCanId = 2;
    public int kRearRightDrivingCanId = 3;

    public int kFrontLeftTurningCanId = 5;
    public int kRearLeftTurningCanId = 8;
    public int kFrontRightTurningCanId = 6;
    public int kRearRightTurningCanId = 7;

    public boolean kGyroReversed = false;

    public DriveArgs(Boolean simulation) {
        this.simulation = simulation;
    }

    
}
