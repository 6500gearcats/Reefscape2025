// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.GroundIntakeConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntake extends SubsystemBase {

  /** Creates a new GroundIntake. */
  private SparkMax m_flipMotor;
  private DigitalInput m_flipSwitch;
  private SparkMax m_spinMotor;
  private double fake_flipMotor_posititon = 0;
  private double fake_flipMotor_speed = 0;
  private boolean fake_flip_Intake_flipSwitch = false;

  

  public GroundIntake() {
    m_flipSwitch = new DigitalInput(GroundIntakeConstants.kGroundIntakeSwitchPort);

    m_flipMotor = new SparkMax(GroundIntakeConstants.kGroundIntakeFlipperMotorPort, MotorType.kBrushless);
    m_spinMotor = new SparkMax(GroundIntakeConstants.kGroundIntakeMotorPort, MotorType.kBrushless);

    SparkMaxConfig m_spinConfig = new SparkMaxConfig();
    m_spinMotor.configure(m_spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig m_flipConfig = new SparkMaxConfig();
    m_flipMotor.configure(m_flipConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    fake_flipMotor_posititon += fake_flipMotor_speed;
    if(fake_flipMotor_posititon >= 100){ // 100 is fake switch position
      fake_flip_Intake_flipSwitch = true;
    }else{
      fake_flip_Intake_flipSwitch = false;
    }
    SmartDashboard.putNumber("Fake flip In Speed", fake_flipMotor_speed);
    SmartDashboard.putBoolean("Flip Switch", getFlipSwitchValue());
    SmartDashboard.putNumber("Fake Flip In Position",fake_flipMotor_posititon);
    SmartDashboard.putBoolean("Fake_In_FlipSwitch",fake_flip_Intake_flipSwitch);
  }

  //This assumes that the flip switch will the false when you have 
  //intake to the outside of the robot

  //TODO: Check if the logic is correct
  public void flipIntake(double speed) {
    if(Robot.isSimulation()){
      fake_flipMotor_speed = speed;
    }
    m_flipMotor.set(speed);
  }

  public boolean getFlipSwitchValue() {
    if(Robot.isSimulation()){
      return fake_flip_Intake_flipSwitch;
    }
    return m_flipSwitch.get();
  }

  public void spinIntake(double speed) {
    m_spinMotor.set(speed);
  }
}
