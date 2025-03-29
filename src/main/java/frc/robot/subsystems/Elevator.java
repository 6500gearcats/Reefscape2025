// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  // TODO add correct ids
  private SparkMax m_elevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorPort, SparkLowLevel.MotorType.kBrushless);
  private LaserCan m_elevatorLidar = new LaserCan(ElevatorConstants.kLidarChannel);
  //private DigitalInput m_elevatorTopLimitSwitch = new DigitalInput(ElevatorConstants.kElevatorTopSwitchPort);
  private DigitalInput m_elevatorBottomLimitSwitch = new DigitalInput(ElevatorConstants.kElevatordBottomSwitchPort);
  private DigitalInput m_elevatorSourcePositionSwitch = new DigitalInput(4);
  private RelativeEncoder m_encoder = m_elevatorMotor.getEncoder();
  public static boolean elevatorCorrectingPosition = false;
  public static boolean elevatorTooHigh = false;
  public static boolean elevatorTooHighForTurbo = false;
  public static boolean elevatorTooHighForRegularSpeed = false;
  public String elevatorState = "innactive";
  
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
    SmartDashboard.putNumber("Elevator Height (m)", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator At Bottom", ElevatorAtBottom());
    SmartDashboard.putBoolean("No Turbo", elevatorTooHighForTurbo);
    SmartDashboard.putNumber("Encoder Rotations", m_encoder.getPosition());
    SmartDashboard.putBoolean("Elevator at Source", m_elevatorSourcePositionSwitch.get());
    //SmartDashboard.putBoolean("Height Malfunctioning", !(m_elevatorLidar.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) || m_elevatorLidar.getMeasurement().distance_mm == 0 && !ElevatorAtBottom());
    SmartDashboard.putBoolean("Move Slow", elevatorTooHighForRegularSpeed);
    //SmartDashboard.putBoolean("Elevator Limit Reached", elevatorAtLimit());
    elevatorCorrectingPosition = getElevatorHeight() < 0.16;
    elevatorTooHigh = getElevatorHeight() > .3;
    elevatorTooHighForTurbo = getElevatorHeight() > 0.22;
    elevatorTooHighForRegularSpeed = getElevatorHeight() > 0.26;
  }

  // Return the height of the elevator in meters
  public double getElevatorHeight(){
    if(Robot.isSimulation()){
      // LaserCan.Measurement measurement = new Measurement(0, 0, 0, false, 0, null);
      return 0;
    }
    LaserCan.Measurement measurement = m_elevatorLidar.getMeasurement();
    if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && !(measurement.distance_mm == 0 && !ElevatorAtBottom()))
    {
      return (double)measurement.distance_mm/1000.0;
    } else {
      return m_encoder.getPosition() * ElevatorConstants.kRotationsToMeters;
    }
    //return 0;
  }

  // Set the elevator speed
  public void setElevatorSpeed(double speed){
    if(!(Arm.armCorrectingPosition && elevatorCorrectingPosition)){
      m_elevatorMotor.set(speed);
    } else {
      m_elevatorMotor.set(-0.04);
    }
  }

  public boolean ElevatorAtBottom(){
    return m_elevatorBottomLimitSwitch.get();
  }

  public boolean ElevatorAtSource(){
    return m_elevatorSourcePositionSwitch.get();
  }

  // Check to see if the elevator is too high
  /*public boolean elevatorAtLimit()
  {
    return m_elevatorTopLimitSwitch.get() || m_elevatorBottomLimitSwitch.get();
  }*/
}