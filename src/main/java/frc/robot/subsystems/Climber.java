
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private SparkMax m_armMotor;
  private double fake_arm_speed = 0;
  /** Creates a new Climber. */
  public Climber() {
    m_armMotor = new SparkMax(ClimberConstants.kClimberMotorPort, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Fake Climber Arm Speed",fake_arm_speed);
  }

  public void setArmMotorSpeed(double speed) {
    if(Robot.isSimulation()){
      fake_arm_speed = speed;
    }
    m_armMotor.set(speed);
  }

}
