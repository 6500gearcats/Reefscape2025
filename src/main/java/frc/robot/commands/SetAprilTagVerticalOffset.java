// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAprilTagVerticalOffset extends Command {
  private int fiducialID;
  private Vision camera;
  private DriveSubsystem drive;
  private double yOffset;

  private double yaw ; // Yaw angle from april tag
  private double distance; // Distance from april tag
  private double ydistance; // X distance only from april tag
  private double distanceFromTarget; // X distance from our target x position

  private boolean hasTarget;
  private double temp = 0;
  private long lastTime = 0;
  /** Creates a new MoveToAprilTag. */
  public SetAprilTagVerticalOffset(int fiducialID, Vision camera, DriveSubsystem drive, double xOffset) {
    this.fiducialID = fiducialID;
    this.camera = camera;
    this.drive = drive;
    this.yOffset = xOffset;
    hasTarget = true;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yaw = camera.getChosenYaw(fiducialID); // Yaw angle from april tag
    distance = camera.getChosenRange(fiducialID); // Distance from april tag
    double ydistance = distance * Math.cos(yaw*Math.PI/180); // y distance only from april tag
   double distanceFromTarget = yOffset - ydistance; // X distance from our target x position

    if(ydistance == 0) {
      cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(camera.getChosenRange(fiducialID) == 0)) {
      
      // All of this is relative to the target april tag
      yaw = camera.getChosenYaw(fiducialID); // Yaw angle from april tag
      distance = camera.getChosenRange(fiducialID); // Distance from april tag
      ydistance = distance * Math.cos(yaw*Math.PI/180); // X distance only from april tag
      distanceFromTarget = yOffset - ydistance; // X distance from our target x position
      lastTime = System.currentTimeMillis();
    } else {
      MoveToAprilTag();
    }

    // Drive to get to the right x distance from the april tag
    drive.drive(distanceFromTarget * 0.5, 0, 0, false);
    temp = ydistance;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(distanceFromTarget) < .2;
  }

  public void MoveToAprilTag() {
    long time = (long) (temp*100) * 2;
    if((System.currentTimeMillis() - lastTime) >= time) {
      this.cancel();
    }
    else {
      drive.drive(0.5, 0, 0, false);
    }
  }
}
