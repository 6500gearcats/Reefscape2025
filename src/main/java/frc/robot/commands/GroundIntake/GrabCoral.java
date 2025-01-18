// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabCoral extends Command {
  /** Creates a new GrabCoral. */
  GroundIntake m_groundIntake;
  double speed;
  public GrabCoral(GroundIntake groundIntake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_groundIntake = groundIntake;
    this.speed = speed;
    addRequirements(m_groundIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_groundIntake.spinIntake(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: Add real speed
    m_groundIntake.spinIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntake.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
