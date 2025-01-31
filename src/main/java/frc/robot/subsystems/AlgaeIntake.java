// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  SparkMax m_intakeMotor;
  DigitalInput m_intakeSwitch;
  double m_fakeSpeed = 0;
  boolean m_fakeSwitch = false;
  double m_fakeAlgaePosition = 0;

  public AlgaeIntake() {
    m_intakeMotor = new SparkMax(0, MotorType.kBrushless);
    m_intakeSwitch = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Robot.isSimulation()){
      SmartDashboard.putNumber("Algae Intake Speed", m_fakeSpeed);
      SmartDashboard.putBoolean("Algae Intak Switch", m_fakeSwitch);
      m_fakeAlgaePosition += m_fakeSpeed;
    }
  }

  public void intakeAlgae(double speed) {
    m_intakeMotor.set(speed);

    if(Robot.isSimulation()){
      m_fakeSpeed = speed/10;
    }
  }

  public boolean hasAlgae() {
    if(Robot.isReal()){
      return m_intakeSwitch.get();
    }
    return m_fakeAlgaePosition > 1;
  }
}