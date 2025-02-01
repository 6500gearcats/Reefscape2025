// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlipGroundIntakeOut extends Command {
  final double speed;
  GroundIntake m_groundIntake;
  boolean m_finished = false;
  
  /** Creates a new FlipGroundIntake. */
  public FlipGroundIntakeOut(GroundIntake groundIntake, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_groundIntake = groundIntake;
    addRequirements(m_groundIntake);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_groundIntake.flipIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntake.flipIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
