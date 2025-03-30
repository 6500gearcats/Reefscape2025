// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;

public class ProportionalAlignHelper  {
private static AprilTagFieldLayout field = VisionConstants.kTagLayout;

/** 
 * Takes the robot position with cooresponding 
 * x/y offsets from the april tag position, returns a pose2d for where robot should go
*/
public static Pose2d getBestAprilTag(Pose2d robotPose, double m_xOffset, double m_yOffset) {
  // Gets the tag of least distance through below helper method
  int bestAprilTag = getClosestAprilTagID(robotPose.getTranslation());
  // Gets the tag pose for tag of least distance int using wpilib field
  Pose2d tagPose = field.getTagPose(bestAprilTag).get().toPose2d();

  // Gets the angle of tag pose
  double tempAngle = tagPose.getRotation().getRadians();
  // Gets the tag's x and y position and multiplies them by offsets so that it's more backwards and to either side
  double newX = tagPose.getX() + Math.cos(tempAngle) * m_yOffset + Math.cos(tempAngle + Math.PI / 2) * m_xOffset;
  double newY = tagPose.getY() + Math.sin(tempAngle) * m_yOffset + Math.sin(tempAngle + Math.PI / 2) * m_xOffset;

  // Returns all of the values in a pose2d (flips the angle to have the robot align without turning around)
  return new Pose2d(newX, newY, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
}


/** 
 * Helper method for getting the best april tag's offset: 
 * Takes robot pose and outputs integer of best april tag
*/
private static int getClosestAprilTagID(Translation2d robotPose) {

  // Initializing integer of best april tag that will be returned
  int integer = 0;

  // Parallel ArrayList - one for the double distance and one for the tag int
  ArrayList<Double> distances = new ArrayList<>();
  ArrayList<Integer> tags = new ArrayList<>();

  // Logic for getting the reef tag ids based on the alliance
  Optional<Alliance> alliance = DriverStation.getAlliance();
  if (alliance.isPresent()) {
    // Tags 6-12 for red alliance, else tags 17-23 for blue alliance
    int startTag = (alliance.get() == Alliance.Red) ? 6 : 17;
    int endTag = (alliance.get() == Alliance.Red) ? 12 : 23;

    // Loops through all of the reef tags
    for (int i = startTag; i < endTag; i++) {
      // Skip tags 7, 10, and 21
      // if(i == 7 || i == 10 || i == 21){
      //   i++;
      // }
      // Gets the position of the reef tag using the field layout provided by WPILib
      Translation2d tagPose = field.getTagPose(i).get().getTranslation().toTranslation2d();
      // Puts the distance between the robot and tag position into distance arrayList
      double distance = robotPose.getDistance(tagPose);
      distances.add(distance);
      // Stores the tag id into the parallel arraylist
      tags.add(i);
    }
  } 

  // If alliance isn't present for whatever reason just go to the nearest april tag (shouldn't ever really happen) - same logic as above
  else {
    for (int i = 1; i < 23; i++) {
      Translation2d tagPose = field.getTagPose(i).get().getTranslation().toTranslation2d();
      double distance = robotPose.getDistance(tagPose);
      distances.add(distance);
      tags.add(i);
    }
  }

  // Takes the minimum of the distances arraylist
  double minDistance = Collections.min(distances);
  // Gets the index of that least distance
  integer = distances.indexOf(minDistance);
  // Returns the tag id using the index of least distance in parallel tag arraylist
  return tags.get(integer);
  }
}
