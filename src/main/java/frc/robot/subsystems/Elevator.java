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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase {
  // TODO add correct ids
  private TalonFX m_elevatorMotor = new TalonFX(Constants.ElevatorConstants.kElevatorMotorPort);
  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

  //private SparkMax m_elevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorPort, SparkLowLevel.MotorType.kBrushless);
  private LaserCan m_elevatorLidar = new LaserCan(ElevatorConstants.kLidarChannel);
  //private DigitalInput m_elevatorTopLimitSwitch = new DigitalInput(ElevatorConstants.kElevatorTopSwitchPort);
  private DigitalInput m_elevatorBottomLimitSwitch = new DigitalInput(ElevatorConstants.kElevatordBottomSwitchPort);
  private DigitalInput m_elevatorSourcePositionSwitch = new DigitalInput(4);
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
    // in init function
var talonFXConfigs = new TalonFXConfiguration();

// set slot 0 gains
var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
slot0Configs.kI = 0; // no output for integrated error
slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

// set Motion Magic Expo settings
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

m_elevatorMotor.getConfigurator().apply(talonFXConfigs);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Elevator Height (m)", getElevatorHeight());
    SmartDashboard.putNumber("ElevatorPositionValues", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator At Bottom", ElevatorAtBottom());
    SmartDashboard.putBoolean("No Turbo", elevatorTooHighForTurbo);
    SmartDashboard.putNumber("Encoder Rotations", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator at Source", m_elevatorSourcePositionSwitch.get());
    //SmartDashboard.putBoolean("Height Malfunctioning", !(m_elevatorLidar.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) || m_elevatorLidar.getMeasurement().distance_mm == 0 && !ElevatorAtBottom());
    SmartDashboard.putBoolean("Move Slow", elevatorTooHighForRegularSpeed);
    //SmartDashboard.putBoolean("Elevator Limit Reached", elevatorAtLimit());
     elevatorCorrectingPosition = getElevatorHeight() > -16.8;
     if (ElevatorAtBottom()) {
      m_elevatorMotor.setPosition(0);
     }
    elevatorTooHigh = getElevatorHeight() < -81;
    elevatorTooHighForTurbo = getElevatorHeight() < -36;
    elevatorTooHighForRegularSpeed = getElevatorHeight() < -65;
  }

  public double getElevatorHeight() {
    return m_elevatorMotor.getPosition().getValueAsDouble();
  }

  // Set the elevator speed
  public void setElevatorSpeed(double speed){
    if(!(Arm.armCorrectingPosition && elevatorCorrectingPosition)){
      m_elevatorMotor.set(speed);
    } else {
      m_elevatorMotor.set(-0.04);
    }
  }

  public void setPosition(double pos) {
    m_elevatorMotor.setControl(m_request.withPosition(pos));
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