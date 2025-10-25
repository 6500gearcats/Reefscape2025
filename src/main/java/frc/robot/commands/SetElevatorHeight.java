// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeight extends Command {
  private Elevator m_elevator;
  private double m_height;
  private double minSpeed;
  private double maxSpeed;
  private boolean useAccurate;

  /** Creates a new SetElevatorHeight. */
  // public SetElevatorHeight(Elevator elevator, double height) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   minSpeed = 0.6;
  //   m_elevator = elevator;
  //   m_height = height;
  //   maxSpeed = -2;

  //   addRequirements(m_elevator);
  //}

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator elevator, double height, double minSpeed, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_height = height;
    this.minSpeed = minSpeed;
    this.maxSpeed = maxSpeed;
    addRequirements(m_elevator);
  }

  public SetElevatorHeight(Elevator elevator, double height) {
    m_elevator = elevator;
    m_height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPosition(m_height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double velocity = maxSpeed * (m_height - m_elevator.getElevatorHeight()) - 0.08;

    // if(Math.abs(velocity) < minSpeed - .08){
    //   velocity = minSpeed * Math.abs(velocity)/velocity -.08;
    // }

    // m_elevator.setElevatorSpeed(velocity); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_elevator.setElevatorSpeed(-0.04);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (m_elevator.getElevatorHeight() < m_height + 10 && m_elevator.getElevatorHeight() > m_height -10) {
        return true;
      }
      return false;
}
}