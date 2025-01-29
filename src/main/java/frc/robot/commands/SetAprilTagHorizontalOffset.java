// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAprilTagHorizontalOffset extends Command {
  int fiducialID;
  Vision camera;
  DriveSubsystem drive;
  double xOffset;
  /** Creates a new MoveToAprilTag. */
  public SetAprilTagHorizontalOffset(int fiducialID, Vision camera, DriveSubsystem drive, double xOffset) {
    this.fiducialID = fiducialID;
    this.camera = camera;
    this.drive = drive;
    this.xOffset = xOffset;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // All of this is relative to the target april tag
    double yaw = camera.getChosenYaw(fiducialID); // Yaw angle from april tag
    double distance = camera.getChosenRange(fiducialID); // Distance from april tag
    double xdistance = distance * Math.sin(yaw*Math.PI/180); // X distance only from april tag
    double distanceFromTarget = xOffset - xdistance; // X distance from our target x position

    // Drive to get to the right x distance from the april tag
    drive.drive(0, -distanceFromTarget * 0.5, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // All of this is relative to the target april tag
    double yaw = camera.getChosenYaw(fiducialID); // Yaw angle from april tag
    double distance = camera.getChosenRange(fiducialID); // Distance from april tag
    double xdistance = distance * Math.sin(yaw*Math.PI/180); // X distance only from april tag
    double distanceFromTarget = xOffset - xdistance; // X distance from our target x position
    return Math.abs(distanceFromTarget) < .2;
  }
}
