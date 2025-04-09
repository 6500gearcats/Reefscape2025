// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private DigitalInput m_noteSensor = new DigitalInput(8);
  private DigitalInput m_noteSwitch = new DigitalInput(9);

  public SparkMax m_intakeMotor = new SparkMax(9, MotorType.kBrushless);

  public Intake() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Switch", getSwitch());
    SmartDashboard.putBoolean("Note Color Sensor", isNoteIn());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    m_intakeMotor.set(speed);
  }
  public boolean getSwitch(){
    return !m_noteSwitch.get();
  }
  public boolean isNoteIn(){
    return !m_noteSensor.get();
  }

}
