
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHolder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveCoral extends Command {
  /** Creates a new CoralHolder. */
  CoralHolder m_CoralHolder;
  Double speed;
  boolean upToSpeed = false;
  boolean isIntake = false;
  public MoveCoral(CoralHolder coralHolder, double speed, boolean isIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CoralHolder = coralHolder;
    this.isIntake = isIntake;
    this.speed = speed;
    addRequirements(m_CoralHolder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CoralHolder.shootCoral(speed);

    if(m_CoralHolder.getHolderSpeed() > 200){
        upToSpeed = true;
    }

    m_CoralHolder.m_upToSpeed = upToSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralHolder.shootCoral(0);
    upToSpeed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return upToSpeed && m_CoralHolder.getHolderSpeed() < 150;
   }
}
