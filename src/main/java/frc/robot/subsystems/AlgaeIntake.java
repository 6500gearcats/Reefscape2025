// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  SparkMax m_intakeMotor;
  DigitalInput m_intakeSwitch;
  public AlgaeIntake() {
    m_intakeMotor = new SparkMax(0, MotorType.kBrushless);
    m_intakeSwitch = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeAlgae(double speed) {
    m_intakeMotor.set(speed);
  }

  public boolean hasAlgae() {
    return m_intakeSwitch.get();
  }
}
