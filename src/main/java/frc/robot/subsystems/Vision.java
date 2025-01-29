// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.net.ssl.TrustManagerFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GCLimelight;
import frc.robot.GCPhotonVision;

public class Vision extends SubsystemBase {
  private boolean isLimelight;
  private GCLimelight limelight;
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
  public Vision(GCLimelight theLimelight) {
    isLimelight = true;
    photonCam = null;
    limelight = theLimelight;
  }

  // Side to side, left to right
  public double getBestYaw(){
    if(isLimelight){
      return limelight.getYawDegrees();
    }
    else{
      return photonCam.getBestTargetRotation();
    }
  }

  // Up and down, top to bottom
  public double getBestPitch(){
    if(isLimelight){
      return limelight.getPitchDegrees();
    } else {
      return photonCam.getBestTargetPitch();
    }
  }

  // Might be just a bit shady with limelights
  // Returns your distsance from a target
  public double getBestRange(){
    if(isLimelight){
      return limelight.getTargetDistanceX();
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
      return 0;
    }
    else {
      return photonCam.getBestAprilTag3dData().getRotation().getZ();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Target 17 Z-Axis Rotation", getZAxisRotation(17));
    SmartDashboard.putNumber("Target 17 Yaw", getChosenYaw(17));

  }
}
