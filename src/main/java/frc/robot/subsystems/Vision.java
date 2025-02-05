// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import javax.net.ssl.TrustManagerFactory;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GCLimelight;
import frc.robot.GCPhotonVision;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private boolean isLimelight;
  GCLimelight currentLimelight;
  private GCLimelight limelight;
  private GCLimelight limelight2;
  private GCPhotonVision photonCam;
  /** Creates a new Vision. */

  // IMPORTANT: The following code uses an overflow constructor
  // to adapt the code for the type of camera we're using.

  // PhotonVision camera code
  public Vision(GCPhotonVision photonvision) {
    isLimelight = false;
    limelight = null;
    photonCam = photonvision;
  }

  // Limelight camera code
  public Vision(GCLimelight theLimelight, GCLimelight ll2) {
    isLimelight = true;
    photonCam = null;
    limelight = theLimelight;
    limelight2 = ll2;
    currentLimelight = limelight;
  }

  public void useLL1() {
    if(isLimelight){
      currentLimelight = limelight;
    }
  }
  public void useLL2() {
    if(isLimelight){
    currentLimelight = limelight2;
    }
  }

  public String getName() {
    if(isLimelight){
      return currentLimelight.getName();
    }
    return "";
  }

  // Side to side, left to right
  public double getBestYaw(){
    if(isLimelight){
      return currentLimelight.getYawDegrees();
    }
    else{
      return photonCam.getBestTargetRotation();
    }
  }

  // Up and down, top to bottom
  public double getBestPitch(){
    if(isLimelight){
      return currentLimelight.getPitchDegrees();
    } else {
      return photonCam.getBestTargetPitch();
    }
  }

  // Might be just a bit shady with limelights
  // Returns your distsance from a target
  public double getBestRange(){
    if(isLimelight){
      return currentLimelight.getTargetDistanceX();
    } else {
      return photonCam.getBestTargetRange();
    }
  }

  public double getChosenYaw(int fiducialID)
  {
    if(isLimelight){
      return limelight.getChosenTargetYawDegrees(fiducialID);
    } else {
      return photonCam.getChosenTargetRotation(fiducialID);
    }
  }

  public double getChosenPitch(int fiducialID)
  {
    if(isLimelight){
      return limelight.getChosenTargetPitchDegrees(fiducialID);
    } else {
      return photonCam.getChosenTargetPitch(fiducialID);
    }
  }

  //TODO: Fix Skew
  public double getChosenSkew(int fiducialID)
  {
    if(isLimelight){
      return limelight.getChosenTargetSkewDegrees(fiducialID);
    } else {
      return photonCam.getChosenTargetSkew(fiducialID);
    }
  }

  public double getChosenRange (int fiducialID)
  {
    if(isLimelight){
      return limelight.getChosenTargetDistanceX(fiducialID);
    } else {
      return photonCam.getChosenTargetRange(fiducialID);
    }
  }

  public double getZAxisRotation(int fiducialID) {
    if (isLimelight) {
      return limelight.getChosenTargetSkewDegrees(fiducialID);
    }
    else {
      return photonCam.getAprilTag3dData(fiducialID).getRotation().getZ();
    }
  }

  public double getBestZAxisRotation() {
    if (isLimelight) {
      return limelight2.getBestSkewDegrees();
    }
    else {
      return photonCam.getBestAprilTag3dData().getRotation().getZ();
    }
  }

  public double getTargetDistance() {
    if (isLimelight) {
      return limelight.getTargetDistanceX();
    }
    else {
      return photonCam.getBestTargetRange();
    }
  }

  public boolean isLimelight() {
    return isLimelight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Target Z-Axis Rotation", getBestZAxisRotation());
    SmartDashboard.putNumber("Target Yaw", getBestYaw());
    SmartDashboard.putNumber("Target Distance", getBestRange());
  }
}
