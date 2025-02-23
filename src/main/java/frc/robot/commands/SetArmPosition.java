
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetArmPosition extends Command {
  /** Creates a new SetArmPOsition. */
  Arm m_arm;
  double position;
  public SetArmPosition(Arm m_arm, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.position = position;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = (position - m_arm.getArmPosition()) * -0.95;

    if(Math.abs(velocity) < .35){
      velocity = .35 * Math.abs(velocity)/velocity;
    }

    m_arm.spinArm(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.spinArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(position - m_arm.getArmPosition()) < 0.01;
  }
}
