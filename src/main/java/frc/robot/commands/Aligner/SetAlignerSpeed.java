// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aligner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Aligner;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlignerSpeed extends Command {
  /** Creates a new SetAlignerSpeed. */
  private Aligner m_aligner;
  private double speed;

  public SetAlignerSpeed(Aligner theAligner, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_aligner = theAligner;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_aligner.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_aligner.setSpeed(0);
  }
}
