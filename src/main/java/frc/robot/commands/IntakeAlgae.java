
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  AlgaeIntake m_algaeIntake;
  double speed;
  /** Creates a new IntakeAlgae. */
  public IntakeAlgae(AlgaeIntake m_algaeIntake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_algaeIntake = m_algaeIntake;
    this.speed = speed;
    addRequirements(m_algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_algaeIntake.intakeAlgae(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_algaeIntake.intakeAlgae(-0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
