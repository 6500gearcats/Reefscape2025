// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  SparkMax m_armMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMaxConfig m_config = new SparkMaxConfig();
  SparkAbsoluteEncoder m_armMotorEncoder = m_armMotor.getAbsoluteEncoder();
  double armPosition;

  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //It's progamming's fault (:
    armPosition = getArmPosition();
    SmartDashboard.putNumber("Arm Position", armPosition);
  }

  public void spinArm(double speed) {
    m_armMotor.set(speed);
  }

  public double getArmPosition() {
    //Rotations to degrees
    //return (int) (m_armMotorEncoder.getPosition() % 10) % 360;
    return m_armMotorEncoder.getPosition();
  }
}
