// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.DriveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class RunCoralLeft extends ParallelDeadlineGroup {


    AprilTagFieldLayout field;
    DriveSubsystem m_drive;
  public RunCoralLeft(DriveSubsystem m_drive) {
      super(new IsRobotAtPosition(m_drive));
      this.m_drive = m_drive;
      System.out.println("RunCoral Command Initialized");
      addCommands(new PathfindingCommand(m_drive));
  }


public class PathfindingCommand extends Command {
  public PathfindingCommand(DriveSubsystem m_drive) {
      addRequirements(m_drive);
      System.out.println("PathfindingCommand Created");
  }


  @Override
  public void initialize() {
      System.out.println("Pathfinder Initialized");
  }


  @Override
  public void execute() {
      System.out.println("Doing the Pathfinding");
      Pose2d robotPose = m_drive.getPose();
      Pose2d targetPose = getBestAprilTag(robotPose);
      System.out.println("Target Pose: " + targetPose);
      AutoBuilder.pathfindToPose(targetPose, new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))).schedule();;
    }
 


  @Override
  public void end(boolean interrupted) {
      // TODO: Unsure if this code ever runs, will require testing but this should fix the angle being always slightly offset
    System.out.println("Ended pathfinding");
    new InstantCommand(() -> m_drive.drive(0, 0, 0.2, false)).withTimeout(0.2).schedule();
      System.out.println("Turned slightly");
  }


  @Override
  public boolean isFinished() {
      return false;
  }
    }
    private Pose2d getBestAprilTag(Pose2d robotPose) {
        field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        int bestAprilTag = getClosestAprilTagID(robotPose.getTranslation());
        Pose2d tagPose = field.getTagPose(bestAprilTag).get().toPose2d();


        double tempAngle = tagPose.getRotation().getRadians();
        double newX = tagPose.getX() + Math.cos(tempAngle) * 0.75 - Math.cos(tempAngle + Math.PI / 2) * 0.075;
        double newY = tagPose.getY() + Math.sin(tempAngle) * 0.75 - Math.sin(tempAngle + Math.PI / 2) * 0.075;


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



