
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CoralHolderConstants;

public class CoralHolder extends SubsystemBase {
  /** Creates a new CoralHolder. */
  SparkMax m_coralMotor;
  DigitalInput m_flipSwitch;
  double fake_coral = 0;
  double fake_coral_speed = 0;
  RelativeEncoder m_RelativeEncoder;
  public CoralHolder() {
    m_coralMotor = new SparkMax(CoralHolderConstants.kCoralHolderMotorPort, MotorType.kBrushless);
    m_flipSwitch = new DigitalInput(0);
    m_RelativeEncoder  = m_coralMotor.getEncoder();
  }

  public void shootCoral(double speed) {
    if(Robot.isSimulation()){
      fake_coral_speed = speed; 
    }
    m_coralMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Robot.isSimulation()){
    fake_coral += fake_coral_speed;

    SmartDashboard.putNumber("Fake Coral Position", fake_coral);
    SmartDashboard.putNumber("Fake Coral Speed", fake_coral_speed);
    }
  }

  //True when coral Present
  public double getHolderSpeed(){
    return m_RelativeEncoder.getVelocity();
  }
}
;
