// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // TODO add correct ids
  public static boolean elevatorTooHigh = false;
  private SparkMax m_elevatorMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
  private LaserCan m_elevatorLidar = new LaserCan(0);
  private DigitalInput m_elevatorTopLimitSwitch = new DigitalInput(0);
  private DigitalInput m_elevatorBottomLimitSwitch = new DigitalInput(0);

  
  /** Creates a new Elevator. */
  public Elevator() {
    try {
      m_elevatorLidar.setRangingMode(LaserCan.RangingMode.SHORT);
      m_elevatorLidar.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      m_elevatorLidar.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } 
    catch(ConfigurationFailedException e){
      System.out.println("So... Uh... The laser didn't work. Seth error." + e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Height (m)", this.getElevatorHeight());
    SmartDashboard.putBoolean("Elevator Limit Reached", elevatorAtLimit());
    //TODO Add real values
    elevatorTooHigh = getElevatorHeight() > 1.0;
    Arm.correctingPositionElevator = getElevatorHeight() <1.0;
  }

  // Return the height of the elevator in meters
  public double getElevatorHeight(){
    LaserCan.Measurement measurement = m_elevatorLidar.getMeasurement();
    if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
    {
      return (double)measurement.distance_mm/1000.0;
    }
    return 0;
  }

  // Set the elevator speed
  public void setElevatorSpeed(double speed){
    if(!(Arm.correctingPositionArm && Arm.correctingPositionElevator)){
      m_elevatorMotor.set(speed);
    }
  }

  // Check to see if the elevator is too high
  public boolean elevatorAtLimit()
  {
    return m_elevatorTopLimitSwitch.get() || m_elevatorBottomLimitSwitch.get();
  }
}