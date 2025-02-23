// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeight extends Command {
  private Elevator m_elevator;
  private double m_height;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator elevator, double height) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_height = height;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = -2 * (m_height - m_elevator.getElevatorHeight()) - 0.04;

    if(Math.abs(velocity) < 0.4 - .04){
      velocity = .4 * Math.abs(velocity)/velocity -.04;
    }

    m_elevator.setElevatorSpeed(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setElevatorSpeed(-0.04);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_elevator.ElevatorAtBottom() && -2 * (m_height - m_elevator.getElevatorHeight()) - 0.1 > 0) || Math.abs(m_height - m_elevator.getElevatorHeight()) <0.02;//m_elevator.elevatorAtLimit();
  }
}
