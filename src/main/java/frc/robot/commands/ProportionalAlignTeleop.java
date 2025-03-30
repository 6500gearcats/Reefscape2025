// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.ProportionalAlignHelper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProportionalAlignTeleop extends Command {
  /** Creates a new ProportionalAlign. */
  Pose2d targetPose;
  double dx;
  double dy;
  double dr;
  double m_xOffset;
  double m_yOffset;
  double m_speedModifier;
  double targetX;
  double targetY;
  double targetAngle;
  int drModifier = 50;
  double baseVelocity = 0.0;
  double addAngle = 0.0;

  DriveSubsystem m_drive;

  public ProportionalAlignTeleop(DriveSubsystem drive, double xOffset, double yOffset) {
    m_speedModifier = 1;
    m_xOffset = xOffset;
    m_yOffset = yOffset;
    m_drive = drive;

    addRequirements(m_drive);
  }

  // Overloaded constructor with a speed modifier to use in autos (we need to align faster)
  public ProportionalAlignTeleop(DriveSubsystem drive, double xOffset, double yOffset, double speedModifier) {
    m_speedModifier = speedModifier;
    m_xOffset = xOffset;
    m_yOffset = yOffset;
    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Gets nearest april tag target position through helper class
    targetPose = ProportionalAlignHelper.getBestAprilTag(m_drive.getPose(), m_xOffset, m_yOffset);

    // Gets x, y, and rotation values from april tag
    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetAngle = targetPose.getRotation().getDegrees();

    // Gets the current alliance set in driver station
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get().equals(Alliance.Blue)) {
        baseVelocity = 1;
        addAngle = 0.0;
      } 
      // If alliance is red then add 180 to the targetAngle and multiply velociy by -1 (robot just goes backwards without this on red side)
      else {
        baseVelocity = -1;
        addAngle = 180.0;
      }
    }

    // Modifies the target angle based on alliance, 
    targetAngle = targetAngle - (addAngle * Math.abs(targetAngle)/targetAngle);

    m_drive.targetrotation = targetAngle;
  }

  @Override
  public void execute() {
    // Gets the error values of x direction, y direction
    dx = targetX - m_drive.getPose().getX();
    dy = targetY - m_drive.getPose().getY();

    // Logics to mofidy the targetAngle- localizes the angle to between -180 and 180 and take most efficient path in a very complicated way
    dr = targetAngle - (Math.abs((m_drive.getAngle() % 360)) * (m_drive.getAngle()/Math.abs(m_drive.getAngle())) - 180 * (m_drive.getAngle()/Math.abs(m_drive.getAngle())));
    //dr = (Math.abs(dr) -180) * (Math.abs(dr)/dr);

    // Takes the total sum of errors of x and y direction to use for slowing down the robot
    double total = Math.abs(dx) + Math.abs(dy);

    // Posts these error values to drive subsystem
    m_drive.distanceX = dx;
    m_drive.distanceY = dy;
    m_drive.distanceR = dr;

    // Slows down the error based on the sum of the two errors
    double xRat = dx / total;
    double yRat = dy / total;

    // If errors for x/y positions are too tiny they just become 0
    if (Math.abs(dx) < 0.05) {
      dx = 0;
      xRat = 0;
    }

    if (Math.abs(dy) < 0.05) {
      dy = 0;
      yRat = 0;
    }

    // If rotational speed is less than constant then set it to a minimum speed
    if (Math.abs(dr) / drModifier < 0.05) {
      dr = 0.05 * (dr / Math.abs(dr));
      dr *= drModifier;
    }

    double velocityX = dx * 0.8 * m_speedModifier * baseVelocity;
    double velocityY = dy * 0.8 * m_speedModifier * baseVelocity;

    //double robotOrientedRotation = targetAngle;
    //double yProportion

    double velocityR = dr / drModifier;

    // Run the robot fast when far away
    if (Math.abs(velocityX) > 3.5 && Math.abs(velocityY) > 3.5) {
      m_drive.drive(xRat * 3 * baseVelocity, yRat * 3 * baseVelocity, dr / drModifier, true, "Proportional Alignment 1");
    } 
    
    // If errors are greater than .4 (but less than 3.5, in other words closer to the april tag) run slower
    else if (Math.abs(dx) * 2.5 * m_speedModifier > .4 && Math.abs(dy) * 2.5 * m_speedModifier > .4) {
      m_drive.drive(velocityX, velocityY, velocityR, true, "Proportional Alignment 2");
    } 
    
    // If any less than .4 for the errors then drive very slow
    else {
      m_drive.drive(xRat * .4 * m_speedModifier * baseVelocity, yRat * .4 * m_speedModifier * baseVelocity, dr / drModifier, true, "Proportional Alignment 3");
    }
  }

  // Stops all driving at the end
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, "Proportional Align End");
  }

  // If the errors for x, y, and rotation are tiny then the command finishes
  @Override
  public boolean isFinished() {
    return Math.abs(dx) < 0.03 && Math.abs(dy) < 0.03 && Math.abs(dr) < 15;
  }
}
