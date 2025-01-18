// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlignerConstants;

public class Aligner extends SubsystemBase {

  private SparkMax m_alignerMotor = new SparkMax(AlignerConstants.kAlignerMotorPort, MotorType.kBrushless);

  /** Creates a new Aligner. */
  public Aligner() {
  }

  @Override
  public void periodic() {
  }

  public void setSpeed(double speed) {
    m_alignerMotor.set(speed);
  }
}
