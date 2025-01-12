// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GroundIntake extends SubsystemBase {
  
  /** Creates a new GroundIntake. */
  private SparkMax m_groundIntakeMotor = new SparkMax(0);
  private SparkMaxConfig m_groundConfig  = new SparkMaxConfig();
  public GroundIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setSpeed(double speed) {
    m_groundIntakeMotor.set(speed);
  }
}
