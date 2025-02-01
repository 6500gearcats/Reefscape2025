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
import frc.robot.Constants.CoralHolderConstants;

public class CoralHolder extends SubsystemBase {
  /** Creates a new CoralHolder. */
  SparkMax m_coralMotor;
  DigitalInput m_flipSwitch;
  boolean fake_flipSwitch;
  double fake_coral = 0;
  double fake_coral_speed = 0;
  public CoralHolder() {
    m_coralMotor = new SparkMax(CoralHolderConstants.kCoralHolderMotorPort, MotorType.kBrushless);
    m_flipSwitch = new DigitalInput(0);
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
    if(fake_coral >= 100){ // 100 is fake switch position (:
      fake_flipSwitch = true;
    }
    SmartDashboard.putNumber("Fake Coral Position", fake_coral);
    SmartDashboard.putNumber("Fake Coral Speed", fake_coral_speed);
    SmartDashboard.putBoolean("Fake Coral Collection Switch", fake_flipSwitch);
    }
  }

  //True when coral Present
  public boolean getFlipSwitchValue() {
    if(Robot.isSimulation()){
      return fake_flipSwitch;
    }else{
      return m_flipSwitch.get();
    }
    // Its programmings fault (;
  }
}
;