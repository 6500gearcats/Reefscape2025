// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  

  public GroundIntake() {
    m_flipSwitch = new DigitalInput(0);

    m_flipMotor = new SparkMax(1, MotorType.kBrushless);
    m_spinMotor = new SparkMax(2, MotorType.kBrushless);

    SparkMaxConfig m_spinConfig = new SparkMaxConfig();
    m_spinMotor.configure(m_spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig m_flipConfig = new SparkMaxConfig();
    m_flipMotor.configure(m_flipConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Flip Switch", getFlipSwitchValue());
  }

  //This assumes that the flip switch will the false when you have 
  //intake to the outside of the robot

  //TODO: Check if the logic is correct
  public boolean flipIntake() {
    if(!getFlipSwitchValue()) {
      m_flipMotor.set(1);
      //turns off motor when switch is triggered
      if(getFlipSwitchValue()) {
        m_flipMotor.set(0);
        return true;
      }
    } else {
      m_flipMotor.set(-1);
      //turns off motor when switch is triggered
      if(!getFlipSwitchValue()) {
        m_flipMotor.set(0);
        return true;
      }
    }
    return false;
  }

  public boolean getFlipSwitchValue() {
    return m_flipSwitch.get();
  }

  public void spinIntake(double speed) {
    m_spinMotor.set(speed);
  }
}
