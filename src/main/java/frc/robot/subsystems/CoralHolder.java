// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralHolderConstants;

public class CoralHolder extends SubsystemBase {
  /** Creates a new CoralHolder. */
  SparkMax m_coralMotor;
  DigitalInput m_flipSwitch;
  public CoralHolder() {
    m_coralMotor = new SparkMax(CoralHolderConstants.kCoralHolderMotorPort, MotorType.kBrushless);
    m_flipSwitch = new DigitalInput(0);
  }

  public void shootCoral(double speed) {
    m_coralMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //True when coral Present
  public boolean getFlipSwitchValue() {
    return m_flipSwitch.get();
  }
}
