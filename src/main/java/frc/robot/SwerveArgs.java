package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveArgs {
    /**
     * The MAXSwerve module can be configured with one of three pinion gears: 12T,
     * 13T, or 14T.
     * This changes the drive speed of the module (a pinion gear with more teeth
     * will result in a
     * robot that drives faster).
     */
    public int kDrivingMotorPinionTeeth = 13;

    /*
     * Invert the turning encoder, since the output shaft rotates in the opposite
     * direction of
     * the steering motor in the MAXSwerve Module.
     */
    public boolean kTurningEncoderInverted = true;

    /**
     * Calculations required for driving motor conversion factors and feed forward
     * 5676 is the free speed in rpm
     */
    public double kDrivingMotorFreeSpeedRps = 5676 / 60;
    public double kWheelDiameterMeters = 0.0762;
    public double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    /**
     * 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
     * teeth on the bevel pinion
     */

    public double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

    public double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
    public double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

    public double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public double kTurningEncoderPositionPIDMinInput = 0; // radians
    public double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public double kDrivingP = 0.04;
    public double kDrivingI = 0;
    public double kDrivingD = 0;
    public double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public double kDrivingMinOutput = -1;
    public double kDrivingMaxOutput = 1;

    public double kTurningP = 1;
    public double kTurningI = 0;
    public double kTurningD = 0;
    public double kTurningFF = 0;
    public double kTurningMinOutput = -1;
    public double kTurningMaxOutput = 1;

    public IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public int kDrivingMotorCurrentLimit = 40; // amps
    public int kTurningMotorCurrentLimit = 20; // amps
}