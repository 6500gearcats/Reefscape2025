// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GCAutoBuilder extends SubsystemBase {
  /** Creates a new GCAutoBuilder. */
  AprilTagFieldLayout field;
  DriveSubsystem m_robotDrive;
  public GCAutoBuilder(DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public Command pathFindToPose() {
    return AutoBuilder.pathfindToPose(getBestAprilTag(), new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }
  public Command pathFindToPose2() {
    return AutoBuilder.pathfindToPose(getBestAprilTag2(), new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }

  private Pose2d getBestAprilTag() {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    Pose2d pose = m_robotDrive.getPose();
    int bestAprilTag = getClosestAprilTagID(pose.getTranslation());
    Pose2d newPose = field.getTagPose(bestAprilTag).get().toPose2d();
    //System.out.println("Old Poses values" + newPose.getX() + ", " + newPose.getY() + ". Rotation: " + newPose.getRotation());

    double tempAngle = field.getTagPose(bestAprilTag).get().toPose2d().getRotation().getRadians();
    double newX = 0;
    double newY = 0;
    newX = (newPose.getX() + Math.cos(tempAngle) * .66) + Math.cos(tempAngle + Math.PI/2) * .3;
    newY = (newPose.getY() + Math.sin(tempAngle) * .66) + Math.sin(tempAngle + Math.PI/2) * .3;
    Pose2d thirdPose = new Pose2d(newX, newY, newPose.getRotation().plus(new Rotation2d(Math.PI)));
    //System.out.println("New Poses values" + thirdPose.getX() + ", " + thirdPose.getY() + ". Rotation: " + thirdPose.getRotation());
    return thirdPose;
  }

  private Pose2d getBestAprilTag2() {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    Pose2d pose = m_robotDrive.getPose();
    int bestAprilTag = getClosestAprilTagID(pose.getTranslation());
    Pose2d newPose = field.getTagPose(bestAprilTag).get().toPose2d();
    //System.out.println("Old Poses values" + newPose.getX() + ", " + newPose.getY() + ". Rotation: " + newPose.getRotation());

    double tempAngle = field.getTagPose(bestAprilTag).get().toPose2d().getRotation().getRadians();
    double newX = 0;
    double newY = 0;
    newX = (newPose.getX() + Math.cos(tempAngle) * .66) - Math.cos(tempAngle + Math.PI/2) * .3;
    newY = (newPose.getY() + Math.sin(tempAngle) * .66) - Math.sin(tempAngle + Math.PI/2) * .3;
    Pose2d thirdPose = new Pose2d(newX, newY, newPose.getRotation().plus(new Rotation2d(Math.PI)));
    //System.out.println("New Poses values" + thirdPose.getX() + ", " + thirdPose.getY() + ". Rotation: " + thirdPose.getRotation());
    return thirdPose;
  }

  private int getClosestAprilTagID(Translation2d robotPose) {
    int integer = 0;
    ArrayList<Double> poses = new ArrayList<Double>();
    ArrayList<Integer> list = new ArrayList<Integer>();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red) {
          for (int i = 6; i<12; i++) {
            Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
            double distance = robotPose.getDistance(pose);
            poses.add(distance);
            list.add(i);
          }
        }
        if (alliance.get() == Alliance.Blue) {
          double distance = (double)Integer.MAX_VALUE;
          for (int i = 17; i<23; i++) {
            Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
            if(Math.abs(robotPose.getDistance(pose)) < distance){
              distance = robotPose.getDistance(pose);
              integer = i;
            }
            //distance = robotPose.getDistance(pose);
            //poses.add(distance);
            //list.add(i);
          }
        }
    }
    else {
      for (int i = 1; i<23; i++) {
        Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
        double distance = robotPose.getDistance(pose);
        poses.add(distance);
        list.add(i);
      }
    }
    //double minValue = Collections.min(poses);
    //System.out.println("Min value: " + minValue);
    //integer = poses.indexOf(minValue);
    //System.out.println("AprilTag of least distance: " + list.get(integer));
    //m_robotDrive.aprilTag = integer;//list.get(integer);
    return integer;
  }
}
