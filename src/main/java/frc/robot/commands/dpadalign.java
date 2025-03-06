// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class dpadalign extends Command {
  /** Creates a new PathFindToPos. */
  PathConstraints constraints;
  DriveSubsystem m_drive;
  AprilTagFieldLayout field;
  Pose2d targetPose;
  public dpadalign(DriveSubsystem newM_Drive) {
    m_drive = newM_Drive;
    constraints = new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    addRequirements(m_drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (LimelightHelpers.getTV("limelight-gcc")) {
       targetPose = getBestAprilTag(m_drive.getPose());
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.5;
    Pose2d myPose = m_drive.getPose();
    double myX = myPose.getX();
    double myY = myPose.getY();
    
    double poseX = targetPose.getX();
    double poseY = targetPose.getY();

    double xError = (myX - poseX);
    double yError = (myY - poseY);

    double xOutput = xError * kP;
    double yOutput = yError * kP;
    m_drive.drive(xOutput, yOutput, 0 , true);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, interrupted);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double limit = 0.1;
    return m_drive.getPose().getX() - targetPose.getX() < limit && m_drive.getPose().getY() - targetPose.getY() < limit;
  }

  private Pose2d getBestAprilTag(Pose2d robotPose) {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    int bestAprilTag = getClosestAprilTagID(robotPose.getTranslation());
    Pose2d tagPose = field.getTagPose(bestAprilTag).get().toPose2d();

    double tempAngle = tagPose.getRotation().getRadians();
    double newX = tagPose.getX() + Math.cos(tempAngle) * 0.75 + Math.cos(tempAngle + Math.PI / 2) * 0.19;
    double newY = tagPose.getY() + Math.sin(tempAngle) * 0.75 + Math.sin(tempAngle + Math.PI / 2) * 0.19;


    return new Pose2d(newX, newY, tagPose.getRotation());
    //.plus(new Rotation2d(Units.degreesToRadians(5))));
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


