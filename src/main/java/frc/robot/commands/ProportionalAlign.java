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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProportionalAlign extends Command {
  /** Creates a new ProportionalAlign. */
  Pose2d targetPose;
  AprilTagFieldLayout field;
  double dx;
  double dy;
  double dr;
  double m_xOffset;
  double m_yOffset;
  double targetX;
  double targetY;
  double targetAngle;

  DriveSubsystem m_drive;

  public ProportionalAlign(DriveSubsystem drive, double xOffset, double yOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xOffset = xOffset;
    m_yOffset = yOffset;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = getBestAprilTag(m_drive.getPose());
    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetAngle = targetPose.getRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dx = targetX - m_drive.getPose().getX();
    dy = targetY - m_drive.getPose().getY();
    dr = targetAngle - ((m_drive.getAngle() % 360) - 180);
    double total = Math.abs(dx) + Math.abs(dy);

    m_drive.distanceX = dx;
    m_drive.distanceY = dy;
    m_drive.distanceR = dr;

    double xRat = dx / total;
    double yRat = dy / total;

    /*if (Math.abs(dr) > 180) {
      dr %= 360;
      dr -= 180;
    }*/

    if (Math.abs(dx) < 0.05) {
      dx = 0;
      xRat = 0;
    }

    if (Math.abs(dy) < 0.05) {
      dy = 0;
      yRat = 0;
    }

    // if (Math.abs(dr) < 1) {
    //   dr = 0;
    // }

    if (Math.abs(dr) / 130 < 0.05) {
      dr = 0.05 * (dr / Math.abs(dr));
      dr *= 130;
    }

    if (Math.abs(dx) * 1.5 > 3 && Math.abs(dy) * 1.5 > 3) {
      m_drive.drive(xRat * 3, yRat * 3, dr / 130, true);
    } else if (Math.abs(dx) * 1.5 > .4 && Math.abs(dy) * 1.5 > .4) {
      m_drive.drive(dx * 1.5, dy * 1.5, dr / 130, true);
    } else {
      m_drive.drive(xRat * .4, yRat * .4, dr / 130, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(dx) < 0.05 && Math.abs(dy) < 0.05 && Math.abs(dr) < 1;
  }

  private Pose2d getBestAprilTag(Pose2d robotPose) {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    int bestAprilTag = getClosestAprilTagID(robotPose.getTranslation());
    Pose2d tagPose = field.getTagPose(bestAprilTag).get().toPose2d();

    double tempAngle = tagPose.getRotation().getRadians();
    double newX = tagPose.getX() + Math.cos(tempAngle) * m_yOffset + Math.cos(tempAngle + Math.PI / 2) * m_xOffset;
    double newY = tagPose.getY() + Math.sin(tempAngle) * m_yOffset + Math.sin(tempAngle + Math.PI / 2) * m_xOffset;

    return new Pose2d(newX, newY, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
  }

  private int getClosestAprilTagID(Translation2d robotPose) {
    int integer = 0;
    ArrayList<Double> distances = new ArrayList<>();
    ArrayList<Integer> tags = new ArrayList<>();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      int startTag = (alliance.get() == Alliance.Red) ? 6 : 17;
      int endTag = (alliance.get() == Alliance.Red) ? 12 : 23;

      for (int i = startTag; i < endTag; i++) {
        Translation2d tagPose = field.getTagPose(i).get().getTranslation().toTranslation2d();
        double distance = robotPose.getDistance(tagPose);
        distances.add(distance);
        tags.add(i);
      }
    } else {
      for (int i = 1; i < 23; i++) {
        Translation2d tagPose = field.getTagPose(i).get().getTranslation().toTranslation2d();
        double distance = robotPose.getDistance(tagPose);
        distances.add(distance);
        tags.add(i);
      }
    }

    double minDistance = Collections.min(distances);
    integer = distances.indexOf(minDistance);
    return tags.get(integer);
  }

}
