// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorSpeed extends Command {
  private Elevator m_elevator;
  private DoubleSupplier m_speed;
  /** Creates a new SetElevatorSpeed. */
  public SetElevatorSpeed(Elevator elevator, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_speed = speed;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorSpeed(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Set the height to the current height
    m_elevator.setElevatorSpeed(-0.04);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.ElevatorAtBottom() && m_speed.getAsDouble() > 0;//m_elevator.elevatorAtLimit();
  }
}
