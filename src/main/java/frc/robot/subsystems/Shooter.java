// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Intake. */
  //private final DigitalInput m_noteSensor = new DigitalInput(9);
  //private final DigitalInput m_noteSwitch = new DigitalInput(8);

  public SparkMax m_shooterMotor = new SparkMax(10, MotorType.kBrushless);

  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    m_shooterMotor.set(speed);
  }
}
