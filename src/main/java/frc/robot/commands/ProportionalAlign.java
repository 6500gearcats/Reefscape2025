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
  double m_xOffset;
  double m_yOffset;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dx = targetPose.getX() - m_drive.getPose().getX();
    dy = targetPose.getY() - m_drive.getPose().getY();
    double total = Math.abs(dx) + Math.abs(dy);

    double xRat = dx/total;
    double yRat = dy/total;

    m_drive.drive(yRat * 1, xRat * 1, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getBestAprilTag(Pose2d robotPose) {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    int bestAprilTag = getClosestAprilTagID(robotPose.getTranslation());
    Pose2d tagPose = field.getTagPose(bestAprilTag).get().toPose2d();

    double tempAngle = tagPose.getRotation().getRadians();
    double newX = tagPose.getX() + Math.cos(tempAngle) * m_xOffset - Math.cos(tempAngle + Math.PI / 2) * m_yOffset;
    double newY = tagPose.getY() + Math.sin(tempAngle) * m_xOffset - Math.sin(tempAngle + Math.PI / 2) * m_yOffset;

    return new Pose2d(newX, newY, tagPose.getRotation());
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
