// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private SparkMax m_armMotor;
  private SparkMax m_grabberMotor;
  /** Creates a new Climber. */
  public Climber() {
    m_armMotor = new SparkMax(0, MotorType.kBrushless);
    m_grabberMotor = new SparkMax(1, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmMotorSpeed(double speed) {
    m_armMotor.set(speed);
  }
  
  public void setGrabberMotorSpeed(double speed){
    m_grabberMotor.set(speed);
  }


}
